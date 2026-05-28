/*
 * kyocera_pmd_init - Week 6 Task 3 (DPDK 22.11 / LSDK 2108)
 *
 * Extends Week 5 with LDPC encode enqueue to LA1224 over PCIe.
 * Counters:
 *   rx_total            - all packets received from the NIC
 *   rx_test             - packets matching the Kyocera UDP signature
 *   op_alloc_drops      - bbdev op pool exhaustions
 *   ops_enqueued        - LDPC encode ops accepted by the host->modem ring
 *   ops_enqueue_failed  - ops dropped because the ring was full
 *   ops_dequeued        - completed encode ops drained back from LA1224
 *   out_alloc_drops     - output mbuf pool exhaustions
 *
 * LDPC params (locked with Sovan, 5-May-2026):
 *   BG=1, z_c=64, q_m=2 (QPSK), code_rate=1/2 -> E=2*K=2816, RV=0,
 *   CB mode, CRC appended by LA1224 firmware (no host-side CRC flag).
 *
 * Run command:
 *   ./kyocera_pmd_init -l 0-3 -n 1 --file-prefix=kpmd --vdev=baseband_la12xx
 */   
         
#include <stdio.h>     
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <inttypes.h>
#include <signal.h>
#include <stdlib.h>
#include <netinet/in.h>

#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_udp.h>
#include <rte_mbuf.h>
#include <rte_mempool.h>
#include <rte_lcore.h>
#include <rte_cycles.h>
#include <rte_debug.h>
#include <rte_errno.h>
#include <rte_log.h>
#include <rte_byteorder.h>

#include <rte_bbdev.h>
#include <rte_bbdev_op.h>

/*
 * Forward-declare the only la12xx-specific call we need.
 * The full <rte_pmd_bbdev_la12xx.h> pulls in <geul_feca.h> (NXP internal),
 * which isn't on the default include path. This avoids that dependency.
 */
__rte_experimental int
rte_pmd_la12xx_map_hugepage_addr(uint16_t dev_id, void *addr);

#define NB_MBUF              8192
#define MBUF_CACHE_SIZE      256
#define MBUF_DATAROOM        RTE_MBUF_DEFAULT_BUF_SIZE

#define NB_RX_QUEUES         1
#define NB_TX_QUEUES         1
#define NB_RX_DESC           1024
#define NB_TX_DESC           1024

#define BURST_SIZE           32
#define POLL_SECONDS         60                /* longer to capture full burst */

#define NB_BBDEV_QUEUES      2
#define NB_BBDEV_OPS         16
#define BBDEV_OP_POOL_SIZE   2048
#define BBDEV_OP_POOL_CACHE  128

#define STATS_PRINT_SEC      5

#define TEST_UDP_DPORT       5678              /* scapy script dst port */
#define TEST_PAYLOAD_MAGIC   "KyoceraUDP"
#define TEST_PAYLOAD_LEN     10

/* Output mbuf pool for encoded bits (one per enqueued op). */
#define OUTPUT_MBUF_SIZE     RTE_MBUF_DEFAULT_BUF_SIZE
#define OUTPUT_MBUF_COUNT    8192
#define OUTPUT_MBUF_CACHE    256

/* LDPC encode parameters (Sovan-locked, 5-May-2026).
 * BG1 + z_c=64 + n_filler=0 -> K = 22*z_c = 1408 info bits per CB.
 * BG1 mother codeword length N = 66*z_c = 4224 bits.
 * MCS 1 (QPSK 1/2) -> E = K/R = 2*K = 2816 bits.
 */
#define LDPC_BG              1
#define LDPC_Z_C             64
#define LDPC_Q_M             2          /* QPSK */
#define LDPC_RV              0
#define LDPC_CB_MODE         1
#define LDPC_K               (22 * LDPC_Z_C)         /* 1408 bits info  */
#define LDPC_N               (66 * LDPC_Z_C)         /* 4224 bits coded */
#define LDPC_N_CB            LDPC_N                   /* full circ buf  */
#define LDPC_E               (LDPC_K * 2)            /* 2816 bits out  */
#define LDPC_INPUT_BYTES     (LDPC_K / 8)            /* 176 bytes / CB */

#define RTE_LOGTYPE_KPMD     RTE_LOGTYPE_USER1

static volatile sig_atomic_t force_quit;

static void
handle_signal(int sig)
{
    (void)sig;
    force_quit = 1;
}

static struct rte_eth_conf port_conf_default = {
    .rxmode = {
        .mq_mode = RTE_ETH_MQ_RX_NONE,
        .mtu     = RTE_ETHER_MTU,
    },
    .txmode = {
        .mq_mode = RTE_ETH_MQ_TX_NONE,
    },
};

/*
 * Map a mempool's hugepage region into the LA12xx PCIe-reachable window.
 * Without this, FECA's QDMA cannot DMA from/to our mbuf data and faults
 * with QdmaErrorHandler / DECCD register dumps on the modem UART.
 *
 * We allocate one mbuf to get a representative address inside the pool's
 * backing hugepage, hand it to rte_pmd_la12xx_map_hugepage_addr() which
 * maps from the end of that hugepage downward (mempool grows from end).
 */
static int
la12xx_map_pool(struct rte_mempool *mp, uint16_t bbdev_id, const char *name)
{
    struct rte_mbuf *probe = rte_pktmbuf_alloc(mp);
    if (probe == NULL) {
        RTE_LOG(ERR, KPMD, "%s: probe alloc failed for la12xx map\n", name);
        return -1;
    }
    void *probe_addr = probe->buf_addr;
    rte_pktmbuf_free(probe);

    int mapped = rte_pmd_la12xx_map_hugepage_addr(bbdev_id, probe_addr);
    if (mapped < 0) {
        RTE_LOG(ERR, KPMD,
                "%s: la12xx_map_hugepage_addr(%p) failed: %d\n",
                name, probe_addr, mapped);
        return -1;
    }
    RTE_LOG(INFO, KPMD,
            "%s: la12xx mapped %d bytes from hugepage @%p\n",
            name, mapped, probe_addr);
    return 0;
}

static void
log_port_info(uint16_t port_id, const struct rte_eth_dev_info *info)
{
    RTE_LOG(INFO, KPMD,
            "Port %u: driver=%s max_rxq=%u max_txq=%u\n",
            port_id, info->driver_name,
            info->max_rx_queues, info->max_tx_queues);
}

static void
log_mac_and_link(uint16_t port_id)
{
    struct rte_ether_addr mac;
    rte_eth_macaddr_get(port_id, &mac);
    RTE_LOG(INFO, KPMD,
            "Port %u MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
            port_id,
            mac.addr_bytes[0], mac.addr_bytes[1], mac.addr_bytes[2],
            mac.addr_bytes[3], mac.addr_bytes[4], mac.addr_bytes[5]);

    struct rte_eth_link link;
    rte_eth_link_get_nowait(port_id, &link);
    RTE_LOG(INFO, KPMD,
            "Port %u link %s speed %u Mbps duplex %s\n",
            port_id,
            link.link_status ? "UP" : "DOWN",
            link.link_speed,
            link.link_duplex == RTE_ETH_LINK_FULL_DUPLEX ? "full" : "half");
}

/*
 * Returns 1 if the mbuf is a Kyocera test packet (UDP dport 5678 with
 * "KyoceraUDP" magic at start of payload). Returns 0 otherwise.
 */
static int
is_test_packet(struct rte_mbuf *m)
{
    uint32_t total_len = rte_pktmbuf_pkt_len(m);
    if (total_len < sizeof(struct rte_ether_hdr) +
                    sizeof(struct rte_ipv4_hdr) +
                    sizeof(struct rte_udp_hdr) +
                    TEST_PAYLOAD_LEN)
        return 0;

    struct rte_ether_hdr *eh = rte_pktmbuf_mtod(m, struct rte_ether_hdr *);
    if (rte_be_to_cpu_16(eh->ether_type) != RTE_ETHER_TYPE_IPV4)
        return 0;

    struct rte_ipv4_hdr *ip = (struct rte_ipv4_hdr *)(eh + 1);
    if (ip->next_proto_id != IPPROTO_UDP)
        return 0;

    uint8_t ihl = (ip->version_ihl & 0x0f) * 4;
    struct rte_udp_hdr *udp = (struct rte_udp_hdr *)((uint8_t *)ip + ihl);
    if (rte_be_to_cpu_16(udp->dst_port) != TEST_UDP_DPORT)
        return 0;

    char *payload = (char *)(udp + 1);
    if (memcmp(payload, TEST_PAYLOAD_MAGIC, TEST_PAYLOAD_LEN) != 0)
        return 0;

    return 1;
}

static int
configure_port(uint16_t port_id, struct rte_mempool *mp)
{
    struct rte_eth_dev_info dev_info;
    int ret = rte_eth_dev_info_get(port_id, &dev_info);
    if (ret != 0)
        rte_exit(EXIT_FAILURE, "dev_info_get(port=%u): %s\n",
                 port_id, strerror(-ret));

    log_port_info(port_id, &dev_info);

    struct rte_eth_conf port_conf = port_conf_default;
    port_conf.rxmode.offloads &= dev_info.rx_offload_capa;
    port_conf.txmode.offloads &= dev_info.tx_offload_capa;

    ret = rte_eth_dev_configure(port_id, NB_RX_QUEUES, NB_TX_QUEUES, &port_conf);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "dev_configure(port=%u): %d\n", port_id, ret);

    uint16_t nb_rxd = NB_RX_DESC;
    uint16_t nb_txd = NB_TX_DESC;
    ret = rte_eth_dev_adjust_nb_rx_tx_desc(port_id, &nb_rxd, &nb_txd);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "adjust_nb_rx_tx_desc: %d\n", ret);

    int sock = rte_eth_dev_socket_id(port_id);
    if (sock < 0) sock = SOCKET_ID_ANY;

    for (uint16_t q = 0; q < NB_RX_QUEUES; q++) {
        ret = rte_eth_rx_queue_setup(port_id, q, nb_rxd, sock, NULL, mp);
        if (ret < 0)
            rte_exit(EXIT_FAILURE, "rx_queue_setup: %d\n", ret);
    }

    struct rte_eth_txconf txconf = dev_info.default_txconf;
    txconf.offloads = port_conf.txmode.offloads;
    for (uint16_t q = 0; q < NB_TX_QUEUES; q++) {
        ret = rte_eth_tx_queue_setup(port_id, q, nb_txd, sock, &txconf);
        if (ret < 0)
            rte_exit(EXIT_FAILURE, "tx_queue_setup: %d\n", ret);
    }

    ret = rte_eth_dev_start(port_id);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "dev_start: %d\n", ret);

    rte_eth_promiscuous_enable(port_id);

    log_mac_and_link(port_id);
    return 0;
}

static uint16_t
bbdev_init(int *sock_out)
{
    uint16_t nb_bbdevs = rte_bbdev_count();
    RTE_LOG(INFO, KPMD, "Found %u bbdev device(s).\n", nb_bbdevs);
    if (nb_bbdevs == 0)
        rte_exit(EXIT_FAILURE,
                 "No bbdev devices. Did you pass --vdev=baseband_la12xx?\n");

    uint16_t bbdev_id = 0;
    int found = 0;
    int sock = SOCKET_ID_ANY;
    uint16_t i;
    RTE_BBDEV_FOREACH(i) {
        struct rte_bbdev_info bbinfo;
        rte_bbdev_info_get(i, &bbinfo);
        RTE_LOG(INFO, KPMD,
                "bbdev %u: driver=%s socket=%d max_queues=%u "
                "queue_size_lim=%u\n",
                i,
                bbinfo.drv.driver_name,
                bbinfo.socket_id,
                bbinfo.drv.max_num_queues,
                bbinfo.drv.queue_size_lim);
        if (!found) {
            bbdev_id = i;
            sock = (bbinfo.socket_id < 0) ? SOCKET_ID_ANY : bbinfo.socket_id;
            found = 1;
        }
    }

    int ret = rte_bbdev_setup_queues(bbdev_id, NB_BBDEV_QUEUES, sock);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "rte_bbdev_setup_queues: %d\n", ret);

    struct rte_bbdev_queue_conf qconf = {
        .socket          = sock,
        .queue_size      = NB_BBDEV_OPS,
        .priority        = 0,
        .deferred_start  = 0,
        .op_type         = RTE_BBDEV_OP_LDPC_ENC,
    };
    ret = rte_bbdev_queue_configure(bbdev_id, 0, &qconf);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "queue_configure(q=0): %d\n", ret);

    qconf.op_type = RTE_BBDEV_OP_LDPC_DEC;
    ret = rte_bbdev_queue_configure(bbdev_id, 1, &qconf);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "queue_configure(q=1): %d\n", ret);

    RTE_LOG(INFO, KPMD,
            "bbdev %u: %u queues configured (q0=LDPC_ENC, q1=LDPC_DEC)\n",
            bbdev_id, NB_BBDEV_QUEUES);

    ret = rte_bbdev_start(bbdev_id);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "rte_bbdev_start: %d\n", ret);

    RTE_LOG(INFO, KPMD,
            "bbdev %u STARTED. IPC handshake complete. LA1224 FECA ready.\n",
            bbdev_id);

    if (sock_out)
        *sock_out = sock;
    return bbdev_id;
}

/*
 * Wait for link UP before starting the timer (so scapy can race with us).
 * Returns 1 if link came up, 0 if timeout.
 */
static int
wait_for_link_up(uint16_t port_id, unsigned int timeout_sec)
{
    RTE_LOG(INFO, KPMD, "Waiting up to %u s for link UP on port %u...\n",
            timeout_sec, port_id);
    for (unsigned int i = 0; i < timeout_sec * 10; i++) {
        struct rte_eth_link link;
        rte_eth_link_get_nowait(port_id, &link);
        if (link.link_status == RTE_ETH_LINK_UP) {
            RTE_LOG(INFO, KPMD,
                    "Link UP on port %u: %u Mbps %s-duplex\n",
                    port_id, link.link_speed,
                    link.link_duplex == RTE_ETH_LINK_FULL_DUPLEX ? "full" : "half");
            return 1;
        }
        rte_delay_us(100000); /* 100 ms */
    }
    RTE_LOG(WARNING, KPMD, "Link did not come up within %u s.\n", timeout_sec);
    return 0;
}

static void
rx_poll_loop(uint16_t port_id, struct rte_mempool *bbdev_op_pool,
             struct rte_mempool *out_pool, uint16_t bbdev_id)
{
    struct rte_eth_stats s0, s1;
    rte_eth_stats_get(port_id, &s0);

    uint64_t rx_total           = 0;
    uint64_t rx_test            = 0;
    uint64_t op_alloc_drops     = 0;
    uint64_t ops_enqueued       = 0;
    uint64_t ops_enqueue_failed = 0;
    uint64_t ops_dequeued       = 0;
    uint64_t ops_status_err     = 0;
    uint64_t out_alloc_drops    = 0;
    int      first_err_logged   = 0;

    const uint64_t hz       = rte_get_tsc_hz();
    const uint64_t tsc_end  = rte_rdtsc() + (uint64_t)POLL_SECONDS * hz;
    uint64_t tsc_next_print = rte_rdtsc() + (uint64_t)STATS_PRINT_SEC * hz;

    RTE_LOG(INFO, KPMD,
            "Polling port=%u for %u s on lcore %u (Ctrl-C to exit early)...\n",
            port_id, POLL_SECONDS, rte_lcore_id());

    while (!force_quit && rte_rdtsc() < tsc_end) {
        struct rte_mbuf *bufs[BURST_SIZE];
        struct rte_bbdev_enc_op *ops[BURST_SIZE];

        uint16_t n = rte_eth_rx_burst(port_id, 0, bufs, BURST_SIZE);

        if (n > 0) {
            rx_total += n;

            /* Filter: keep only Kyocera test packets, drop the rest */
            struct rte_mbuf *kept[BURST_SIZE];
            uint16_t k = 0;
            for (uint16_t i = 0; i < n; i++) {
                if (is_test_packet(bufs[i])) {
                    kept[k++] = bufs[i];
                    rx_test++;
                } else {
                    rte_pktmbuf_free(bufs[i]);
                }
            }

            /* Allocate bbdev ops, fill LDPC params, enqueue to LA1224 */
            if (k > 0) {
                int ar = rte_bbdev_enc_op_alloc_bulk(bbdev_op_pool, ops, k);
                if (ar != 0) {
                    op_alloc_drops += k;
                    for (uint16_t i = 0; i < k; i++)
                        rte_pktmbuf_free(kept[i]);
                } else {
                    struct rte_bbdev_enc_op *op_arr[BURST_SIZE];
                    uint16_t prepared = 0;

                    for (uint16_t i = 0; i < k; i++) {
                        struct rte_mbuf *out_mb = rte_pktmbuf_alloc(out_pool);
                        if (out_mb == NULL) {
                            rte_pktmbuf_free(kept[i]);
                            rte_bbdev_enc_op_free_bulk(&ops[i], 1);
                            out_alloc_drops++;
                            continue;
                        }
                        struct rte_bbdev_enc_op *op = ops[i];

                        /* LDPC encoder consumes K bits = LDPC_INPUT_BYTES (176)
                         * per code block. Trim if the frame is larger; if it's
                         * smaller, the PMD will read past end-of-data, so we
                         * also pad-up to LDPC_INPUT_BYTES below. */
                        uint16_t in_len = rte_pktmbuf_pkt_len(kept[i]);
                        if (in_len > LDPC_INPUT_BYTES)
                            in_len = LDPC_INPUT_BYTES;

                        /* Zero-copy hugepage input */
                        op->ldpc_enc.input.data    = kept[i];
                        op->ldpc_enc.input.offset  = 0;
                        op->ldpc_enc.input.length  = in_len;

                        /* Output buffer for encoded bits */
                        op->ldpc_enc.output.data   = out_mb;
                        op->ldpc_enc.output.offset = 0;
                        op->ldpc_enc.output.length = 0;

                        /* LDPC parameters (Sovan-locked + n_cb fix) */
                        op->ldpc_enc.basegraph        = LDPC_BG;
                        op->ldpc_enc.z_c              = LDPC_Z_C;
                        op->ldpc_enc.n_cb             = LDPC_N_CB;
                        op->ldpc_enc.q_m              = LDPC_Q_M;
                        op->ldpc_enc.rv_index         = LDPC_RV;
                        op->ldpc_enc.code_block_mode  = LDPC_CB_MODE;
                        op->ldpc_enc.cb_params.e      = LDPC_E;
                        op->ldpc_enc.op_flags         = 0; /* CRC done by LA1224 firmware */

                        op_arr[prepared++] = op;
                    }

                    /* Enqueue with drain-and-retry: when the host->modem
                     * ring is full, dequeue completed ops to make room and
                     * retry the unsubmitted tail. Cap retries so a stuck
                     * modem can't hang the loop. */
                    struct rte_bbdev_enc_op **enq_ptr = op_arr;
                    uint16_t to_enq  = prepared;
                    int      retries = 0;
                    const int RETRY_MAX = 32;

                    while (to_enq > 0 && retries < RETRY_MAX) {
                        uint16_t nb_enq = rte_bbdev_enqueue_enc_ops(
                            bbdev_id, 0, enq_ptr, to_enq);
                        ops_enqueued += nb_enq;
                        enq_ptr      += nb_enq;
                        to_enq       -= nb_enq;
                        if (to_enq == 0)
                            break;

                        /* Ring full: drain whatever's complete, then retry. */
                        struct rte_bbdev_enc_op *deq[BURST_SIZE];
                        uint16_t nb_deq = rte_bbdev_dequeue_enc_ops(
                            bbdev_id, 0, deq, BURST_SIZE);
                        if (nb_deq > 0) {
                            ops_dequeued += nb_deq;
                            for (uint16_t i = 0; i < nb_deq; i++) {
                                if (deq[i]->status != 0) {
                                    ops_status_err++;
                                    if (!first_err_logged) {
                                        RTE_LOG(WARNING, KPMD,
                                            "first op->status nonzero: 0x%x\n",
                                            deq[i]->status);
                                        first_err_logged = 1;
                                    }
                                }
                                rte_pktmbuf_free(deq[i]->ldpc_enc.input.data);
                                rte_pktmbuf_free(deq[i]->ldpc_enc.output.data);
                            }
                            rte_bbdev_enc_op_free_bulk(deq, nb_deq);
                            retries = 0;       /* progress -> reset retries */
                        } else {
                            rte_pause();
                            retries++;
                        }
                    }

                    /* Anything still un-enqueued after RETRY_MAX is dropped. */
                    if (to_enq > 0) {
                        ops_enqueue_failed += to_enq;
                        for (uint16_t i = 0; i < to_enq; i++) {
                            rte_pktmbuf_free(enq_ptr[i]->ldpc_enc.input.data);
                            rte_pktmbuf_free(enq_ptr[i]->ldpc_enc.output.data);
                        }
                        rte_bbdev_enc_op_free_bulk(enq_ptr, to_enq);
                    }
                }
            }
        }

        /* Drain completed encode ops so the modem->host ring stays clear.
         * Task 4 will validate output / CRC; for now we just free. */
        {
            struct rte_bbdev_enc_op *deq_ops[BURST_SIZE];
            uint16_t nb_deq = rte_bbdev_dequeue_enc_ops(bbdev_id, 0,
                                                        deq_ops, BURST_SIZE);
            if (nb_deq > 0) {
                ops_dequeued += nb_deq;
                for (uint16_t i = 0; i < nb_deq; i++) {
                    if (deq_ops[i]->status != 0) {
                        ops_status_err++;
                        if (!first_err_logged) {
                            RTE_LOG(WARNING, KPMD,
                                "first op->status nonzero: 0x%x\n",
                                deq_ops[i]->status);
                            first_err_logged = 1;
                        }
                    }
                    rte_pktmbuf_free(deq_ops[i]->ldpc_enc.input.data);
                    rte_pktmbuf_free(deq_ops[i]->ldpc_enc.output.data);
                }
                rte_bbdev_enc_op_free_bulk(deq_ops, nb_deq);
            }
        }

        uint64_t now = rte_rdtsc();
        if (now >= tsc_next_print) {
            RTE_LOG(INFO, KPMD,
                    "[tick] rx=%" PRIu64 " (test=%" PRIu64 ")"
                    " enq=%" PRIu64 " enq_fail=%" PRIu64
                    " deq=%" PRIu64 " status_err=%" PRIu64
                    " op_alloc_drops=%" PRIu64
                    " out_drops=%" PRIu64 "\n",
                    rx_total, rx_test,
                    ops_enqueued, ops_enqueue_failed, ops_dequeued,
                    ops_status_err,
                    op_alloc_drops, out_alloc_drops);
            tsc_next_print = now + (uint64_t)STATS_PRINT_SEC * hz;
        }
    }

    rte_eth_stats_get(port_id, &s1);

    /* Final drain: pick up anything still in flight at exit. */
    {
        struct rte_bbdev_enc_op *deq_ops[BURST_SIZE];
        for (int pass = 0; pass < 64; pass++) {
            uint16_t nb_deq = rte_bbdev_dequeue_enc_ops(bbdev_id, 0,
                                                        deq_ops, BURST_SIZE);
            if (nb_deq == 0) break;
            ops_dequeued += nb_deq;
            for (uint16_t i = 0; i < nb_deq; i++) {
                rte_pktmbuf_free(deq_ops[i]->ldpc_enc.input.data);
                rte_pktmbuf_free(deq_ops[i]->ldpc_enc.output.data);
            }
            rte_bbdev_enc_op_free_bulk(deq_ops, nb_deq);
        }
    }

    RTE_LOG(INFO, KPMD,
            "Done. rx_total=%" PRIu64 " rx_test=%" PRIu64
            " enq=%" PRIu64 " enq_fail=%" PRIu64 " deq=%" PRIu64
            " op_alloc_drops=%" PRIu64 " out_drops=%" PRIu64
            " | NIC ipackets delta=%" PRIu64
            " imissed delta=%" PRIu64 "\n",
            rx_total, rx_test,
            ops_enqueued, ops_enqueue_failed, ops_dequeued,
            op_alloc_drops, out_alloc_drops,
            s1.ipackets - s0.ipackets,
            s1.imissed  - s0.imissed);
}

int
main(int argc, char **argv)
{
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    int ret = rte_eal_init(argc, argv);
    if (ret < 0)
        rte_panic("EAL init failed: %s\n", rte_strerror(rte_errno));
    argc -= ret;
    argv += ret;

    uint16_t nb_ports = rte_eth_dev_count_avail();
    if (nb_ports == 0)
        rte_exit(EXIT_FAILURE,
                 "No Ethernet ports. Is DPRC set and vfio-fsl-mc bound?\n");
    RTE_LOG(INFO, KPMD, "Found %u Ethernet port(s).\n", nb_ports);

    struct rte_mempool *mbuf_pool = rte_pktmbuf_pool_create(
        "KYOCERA_MBUF_POOL",
        NB_MBUF, MBUF_CACHE_SIZE,
        0, MBUF_DATAROOM,
        rte_socket_id());
    if (mbuf_pool == NULL)
        rte_exit(EXIT_FAILURE, "pktmbuf_pool_create: %s\n",
                 rte_strerror(rte_errno));

    uint16_t port_id;
    RTE_ETH_FOREACH_DEV(port_id) {
        configure_port(port_id, mbuf_pool);
    }
    uint16_t first_port = 0;
    RTE_ETH_FOREACH_DEV(first_port) { break; }

    int bbdev_sock = SOCKET_ID_ANY;
    uint16_t bbdev_id = bbdev_init(&bbdev_sock);

    /* Make the input mbuf pool reachable to LA12xx QDMA. */
    if (la12xx_map_pool(mbuf_pool, bbdev_id, "KYOCERA_MBUF_POOL") < 0)
        rte_exit(EXIT_FAILURE,
                 "Failed to map KYOCERA_MBUF_POOL into LA12xx window\n");

    struct rte_mempool *bbdev_op_pool = rte_bbdev_op_pool_create(
        "BBDEV_OP_POOL_LDPC_ENC",
        RTE_BBDEV_OP_LDPC_ENC,
        BBDEV_OP_POOL_SIZE,
        BBDEV_OP_POOL_CACHE,
        bbdev_sock);
    if (bbdev_op_pool == NULL)
        rte_exit(EXIT_FAILURE, "bbdev_op_pool_create: %s\n",
                 rte_strerror(rte_errno));
    RTE_LOG(INFO, KPMD,
            "bbdev op pool created (size=%u, cache=%u, socket=%d)\n",
            BBDEV_OP_POOL_SIZE, BBDEV_OP_POOL_CACHE, bbdev_sock);

    struct rte_mempool *out_pool = rte_pktmbuf_pool_create(
        "BBDEV_OUT_POOL",
        OUTPUT_MBUF_COUNT, OUTPUT_MBUF_CACHE,
        0, OUTPUT_MBUF_SIZE,
        rte_socket_id());
    if (out_pool == NULL)
        rte_exit(EXIT_FAILURE, "out_pool create failed: %s\n",
                 rte_strerror(rte_errno));
    RTE_LOG(INFO, KPMD,
            "out mbuf pool created (size=%u, cache=%u)\n",
            OUTPUT_MBUF_COUNT, OUTPUT_MBUF_CACHE);

    /* Make the output mbuf pool reachable to LA12xx QDMA. */
    if (la12xx_map_pool(out_pool, bbdev_id, "BBDEV_OUT_POOL") < 0)
        rte_exit(EXIT_FAILURE,
                 "Failed to map BBDEV_OUT_POOL into LA12xx window\n");

    /* Wait for link before starting the timed poll */
    wait_for_link_up(first_port, 15);

    rx_poll_loop(first_port, bbdev_op_pool, out_pool, bbdev_id);

    rte_bbdev_stop(bbdev_id);
    rte_bbdev_close(bbdev_id);
    RTE_LOG(INFO, KPMD, "bbdev %u stopped and closed\n", bbdev_id);

    rte_mempool_free(bbdev_op_pool);
    rte_mempool_free(out_pool);

    RTE_ETH_FOREACH_DEV(port_id) {
        rte_eth_dev_stop(port_id);
        rte_eth_dev_close(port_id);
    }
    rte_mempool_free(mbuf_pool);
    rte_eal_cleanup();
    return 0;
}
