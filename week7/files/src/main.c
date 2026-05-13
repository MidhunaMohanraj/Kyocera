/*
 * kyocera_pmd_dq - Week 7 Task 4 (DPDK 22.11 / LSDK 2108)
 *
 * Extends Task 3 with:
 *   - Dequeue poll loop (drain encode completions from LA1224)
 *   - Re-encapsulation: encoded payload wrapped in Eth/IPv4/UDP
 *   - TX return path through DPMAC.1
 *   - 60-op batch ceiling (Task 3 NXP QDMA workaround)
 *   
 * Spec: Week7_Task4_Dequeue_ReturnPath_UserManual.docx
 *
 * Review fixes applied vs spec:
 *   - OUT_MBUF_SIZE includes 4096 adj_addr headroom (spec had 512 only)
 *   - output.length set to LDPC_OUT_BYTES not 0 (avoids QDMA fmt error)
 *   - dport filter kept at 5678 (our bench setup, not spec's 5000)
 *   - Test PC IP/MAC updated to 192.168.10.20 / Mac Studio MAC
 *   - BBDEV_ID and QID defined explicitly
 *
 * Run command:
 *   ./kyocera_pmd_dq -l 0-3 -n 1 --file-prefix=kpmd \
 *                    --vdev=baseband_la12xx
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <inttypes.h>
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
#include <rte_byteorder.h>
#include <rte_log.h>
#include <rte_errno.h>
#include <rte_bbdev.h>
#include <rte_bbdev_op.h>
#include <stdlib.h>

/*  la12xx PMD experimental APIs  */
__rte_experimental int
rte_pmd_la12xx_map_hugepage_addr(uint16_t dev_id, void *addr);

__rte_experimental void *
rte_pmd_la12xx_ldpc_enc_adj_addr(void *addr, uint64_t num_bytes);

__rte_experimental uint16_t
rte_pmd_la12xx_queue_input_circ_size(uint16_t dev_id, uint16_t queue_id,
                                     uint32_t input_circ_size);

/*  NIC / DPDK sizing  */
#define NB_MBUF              8192
#define MBUF_CACHE_SIZE      256
#define MBUF_DATAROOM        RTE_MBUF_DEFAULT_BUF_SIZE
#define NB_RX_QUEUES         1
#define NB_TX_QUEUES         1
#define NB_RX_DESC           1024
#define NB_TX_DESC           1024
#define BURST_SIZE           32
#define POLL_SECONDS         600
#define STATS_PRINT_SEC      5

/*  bbdev sizing  */
#define BBDEV_ID             0
#define QID                  0
#define NB_BBDEV_QUEUES      2
#define NB_BBDEV_OPS         16
#define BBDEV_OP_POOL_SIZE   2048
#define BBDEV_OP_POOL_CACHE  128

/*  Task 4: batch ceiling workaround  */
/* Keep below the QDMA ring-wrap fault point (~60-70 ops).
 * Increase only after NXP delivers a PMD fix. */
#define NB_BATCH_CAP         60

/*  Output pool  */
/* OUT_MBUF_SIZE includes 4096-byte headroom required by
 * rte_pmd_la12xx_ldpc_enc_adj_addr() for FECA DMA alignment.
 * Spec had 512 bytes which is insufficient. */
#define NB_OUT_MBUFS         (4 * NB_BBDEV_OPS)
#define OUT_MBUF_SIZE        (RTE_MBUF_DEFAULT_BUF_SIZE + 4096)

/*  LDPC parameters  Kyocera locked 05-May-2026  */
#define LDPC_BG              1          /* BG1                         */
#define LDPC_Z_C             64         /* Lifting size                */
#define LDPC_Q_M             2          /* QPSK                        */
#define LDPC_RV              0          /* RV=0                        */
#define LDPC_CB_MODE         1          /* Code block mode             */
#define LDPC_K               (22 * LDPC_Z_C)      /* 1408 bits        */
#define LDPC_N               (66 * LDPC_Z_C)      /* 4224 bits        */
#define LDPC_N_CB            LDPC_N
#define LDPC_E               (LDPC_K * 2)         /* 2816 bits        */
#define LDPC_INPUT_BYTES     (LDPC_K / 8)         /* 176 bytes        */
/* Output size including 3-byte CB-CRC appended by LA1224 firmware */
#define LDPC_OUT_BYTES       ((LDPC_E + 7) / 8 + 3) /* 355 bytes     */

/*  Return-path addressing  */
/* Update TEST_PC_MAC to match your bench Mac Studio en0 MAC */
#define TEST_PC_IP           RTE_IPV4(192, 168, 10, 20)
#define BOARD_IP             RTE_IPV4(192, 168, 10, 11)
#define RETURN_SRC_PORT      5001
#define RETURN_DST_PORT      4001

/*  Test packet filter  */
#define TEST_UDP_DPORT       5678
#define TEST_PAYLOAD_MAGIC   "KyoceraUDP"
#define TEST_PAYLOAD_LEN     10

/*  Input circular buffer  */
#define LDPC_INPUT_CIRC_SIZE 8192

#define RTE_LOGTYPE_KPMD     RTE_LOGTYPE_USER1

static volatile sig_atomic_t force_quit;
static void handle_signal(int sig) { (void)sig; force_quit = 1; }

/*  Counters  */
static uint64_t rx_total, rx_test;
static uint64_t op_alloc_drops, out_alloc_drops;
static uint64_t ops_enqueued, ops_enqueue_failed;
static uint64_t ops_dequeued, ops_failed;
static uint64_t tx_sent, tx_drop;

/*  Pools (global for tx_reencap_and_send)  */
static struct rte_mempool *g_out_mp;
static uint16_t            g_port_id;

/*  PMD experimental declarations  */
static int
la12xx_map_pool(struct rte_mempool *mp, uint16_t bbdev_id, const char *name)
{
    struct rte_mbuf *probe = rte_pktmbuf_alloc(mp);
    if (!probe) {
        RTE_LOG(ERR, KPMD, "%s: probe alloc failed\n", name);
        return -1;
    }
    void *addr = probe->buf_addr;
    rte_pktmbuf_free(probe);

    int mapped = rte_pmd_la12xx_map_hugepage_addr(bbdev_id, addr);
    if (mapped < 0) {
        RTE_LOG(ERR, KPMD, "%s: map_hugepage failed: %d\n", name, mapped);
        return -1;
    }
    if (mapped == 0)
        RTE_LOG(INFO, KPMD,
                "%s: hugepage @%p already in LA12xx window (OK)\n",
                name, addr);
    else
        RTE_LOG(INFO, KPMD,
                "%s: la12xx mapped %d bytes @%p\n", name, mapped, addr);
    return 0;
}

/*  Packet filter  */
static int
is_test_packet(struct rte_mbuf *m)
{
    if (rte_pktmbuf_pkt_len(m) < sizeof(struct rte_ether_hdr) +
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
    return memcmp(payload, TEST_PAYLOAD_MAGIC, TEST_PAYLOAD_LEN) == 0;
}

/*  TX re-encapsulation and send  */
/* Pull encoded payload from completed op, prepend Eth/IPv4/UDP,
 * transmit through DPMAC.1. */
static void
tx_reencap_and_send(struct rte_bbdev_enc_op *op)
{
    struct rte_mbuf *m   = op->ldpc_enc.output.data;
    uint32_t enc_len     = op->ldpc_enc.output.length;

    if (enc_len == 0) {
        /* PMD did not fill length  use expected size */
        enc_len = LDPC_OUT_BYTES;
        RTE_LOG(DEBUG, KPMD, "output.length=0 from PMD, using %u\n",
                enc_len);
    }

    /* Trim mbuf to encoded payload */
    m->data_len = (uint16_t)enc_len;
    m->pkt_len  = enc_len;

    /* Prepend UDP header */
    struct rte_udp_hdr *uh = (struct rte_udp_hdr *)
                              rte_pktmbuf_prepend(m, sizeof(*uh));
    if (!uh) { tx_drop++; rte_pktmbuf_free(m); return; }
    uh->src_port    = rte_cpu_to_be_16(RETURN_SRC_PORT);
    uh->dst_port    = rte_cpu_to_be_16(RETURN_DST_PORT);
    uh->dgram_len   = rte_cpu_to_be_16((uint16_t)(sizeof(*uh) + enc_len));
    uh->dgram_cksum = 0;

    /* Prepend IPv4 header */
    struct rte_ipv4_hdr *ih = (struct rte_ipv4_hdr *)
                               rte_pktmbuf_prepend(m, sizeof(*ih));
    if (!ih) { tx_drop++; rte_pktmbuf_free(m); return; }
    memset(ih, 0, sizeof(*ih));
    ih->version_ihl   = 0x45;
    ih->total_length  = rte_cpu_to_be_16(
                            (uint16_t)(sizeof(*ih) + sizeof(*uh) + enc_len));
    ih->time_to_live  = 64;
    ih->next_proto_id = IPPROTO_UDP;
    ih->src_addr      = rte_cpu_to_be_32(BOARD_IP);
    ih->dst_addr      = rte_cpu_to_be_32(TEST_PC_IP);
    ih->hdr_checksum  = rte_ipv4_cksum(ih);

    /* Prepend Ethernet header */
    struct rte_ether_hdr *eh = (struct rte_ether_hdr *)
                                rte_pktmbuf_prepend(m, sizeof(*eh));
    if (!eh) { tx_drop++; rte_pktmbuf_free(m); return; }
    rte_eth_macaddr_get(g_port_id, &eh->src_addr);
    /* UPDATE: set to actual Mac Studio en0 MAC before building */
    struct rte_ether_addr test_pc_mac = {{0x1c, 0x1d, 0xd3, 0xe1, 0x16, 0x57}};
    eh->dst_addr   = test_pc_mac;
    eh->ether_type = rte_cpu_to_be_16(RTE_ETHER_TYPE_IPV4);

    /* TX burst */
    struct rte_mbuf *tx_burst[1] = { m };
    uint16_t sent = rte_eth_tx_burst(g_port_id, 0, tx_burst, 1);
    if (sent == 1)
        tx_sent++;
    else {
        tx_drop++;
        rte_pktmbuf_free(m);
    }
}

/*  Dequeue batch  */
static uint16_t
dequeue_batch(uint16_t bbdev_id, uint16_t expected)
{
    struct rte_bbdev_enc_op *deq_ops[NB_BBDEV_OPS];
    uint16_t total       = 0;
    uint32_t idle_cycles = 0;

    while (total < expected) {
        uint16_t got = rte_bbdev_dequeue_enc_ops(bbdev_id, QID,
                                                 deq_ops, NB_BBDEV_OPS);
        if (got == 0) {
            if (++idle_cycles > 30000) {
                RTE_LOG(WARNING, KPMD,
                        "dequeue stalled: total=%u expected=%u\n",
                        total, expected);
                return total;
            }
            rte_delay_us(10);
            continue;
        }
        idle_cycles = 0;
        for (uint16_t i = 0; i < got; i++) {
            if (deq_ops[i]->status != 0) {
                ops_failed++;
                RTE_LOG(WARNING, KPMD,
                        "op->status error: 0x%08x\n",
                        (uint32_t)deq_ops[i]->status);
            } else {
                ops_dequeued++;
            }
            tx_reencap_and_send(deq_ops[i]);
            /* Input mbuf freed after re-encap (output mbuf is the tx buf) */
            rte_pktmbuf_free(deq_ops[i]->ldpc_enc.input.data);
            rte_bbdev_enc_op_free_bulk(&deq_ops[i], 1);
        }
        total += got;
    }
    return total;
}

/*  bbdev init  */
static uint16_t
bbdev_init(int *sock_out)
{
    uint16_t nb = rte_bbdev_count();
    RTE_LOG(INFO, KPMD, "Found %u bbdev device(s).\n", nb);
    if (nb == 0)
        rte_exit(EXIT_FAILURE, "No bbdev. Pass --vdev=baseband_la12xx\n");

    uint16_t bbdev_id = 0;
    int sock = SOCKET_ID_ANY;
    uint16_t i;
    RTE_BBDEV_FOREACH(i) {
        struct rte_bbdev_info info;
        rte_bbdev_info_get(i, &info);
        RTE_LOG(INFO, KPMD,
                "bbdev %u: driver=%s socket=%d max_queues=%u\n",
                i, info.drv.driver_name, info.socket_id,
                info.drv.max_num_queues);
        bbdev_id = i;
        sock = (info.socket_id < 0) ? SOCKET_ID_ANY : info.socket_id;
        break;
    }

    int ret = rte_bbdev_setup_queues(bbdev_id, NB_BBDEV_QUEUES, sock);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "setup_queues: %d\n", ret);

    struct rte_bbdev_queue_conf qconf = {
        .socket         = sock,
        .queue_size     = NB_BBDEV_OPS,
        .priority       = 0,
        .deferred_start = 0,
        .op_type        = RTE_BBDEV_OP_LDPC_ENC,
    };
    ret = rte_bbdev_queue_configure(bbdev_id, 0, &qconf);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "queue_configure(q=0 ENC): %d\n", ret);

    qconf.op_type = RTE_BBDEV_OP_LDPC_DEC;
    ret = rte_bbdev_queue_configure(bbdev_id, 1, &qconf);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "queue_configure(q=1 DEC): %d\n", ret);

    /* Input circular buffer size */
    rte_pmd_la12xx_queue_input_circ_size(bbdev_id, 0, LDPC_INPUT_CIRC_SIZE);
    RTE_LOG(INFO, KPMD, "bbdev %u: 2 queues configured, circ=%u\n",
            bbdev_id, LDPC_INPUT_CIRC_SIZE);

    ret = rte_bbdev_start(bbdev_id);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "bbdev_start: %d\n", ret);

    RTE_LOG(INFO, KPMD,
            "bbdev %u STARTED. IPC handshake complete. FECA ready.\n",
            bbdev_id);

    if (sock_out) *sock_out = sock;
    return bbdev_id;
}

/*  Port configure  */
static struct rte_eth_conf port_conf_default = {
    .rxmode = { .mq_mode = RTE_ETH_MQ_RX_NONE, .mtu = RTE_ETHER_MTU },
    .txmode = { .mq_mode = RTE_ETH_MQ_TX_NONE },
};

static void
configure_port(uint16_t port_id, struct rte_mempool *mp)
{
    struct rte_eth_dev_info info;
    rte_eth_dev_info_get(port_id, &info);

    struct rte_eth_conf conf = port_conf_default;
    conf.rxmode.offloads &= info.rx_offload_capa;
    conf.txmode.offloads &= info.tx_offload_capa;
    rte_eth_dev_configure(port_id, NB_RX_QUEUES, NB_TX_QUEUES, &conf);

    uint16_t nb_rxd = NB_RX_DESC, nb_txd = NB_TX_DESC;
    rte_eth_dev_adjust_nb_rx_tx_desc(port_id, &nb_rxd, &nb_txd);

    int sock = rte_eth_dev_socket_id(port_id);
    if (sock < 0) sock = SOCKET_ID_ANY;

    rte_eth_rx_queue_setup(port_id, 0, nb_rxd, sock, NULL, mp);

    struct rte_eth_txconf txconf = info.default_txconf;
    txconf.offloads = conf.txmode.offloads;
    rte_eth_tx_queue_setup(port_id, 0, nb_txd, sock, &txconf);

    rte_eth_dev_start(port_id);
    rte_eth_promiscuous_enable(port_id);

    struct rte_ether_addr mac;
    rte_eth_macaddr_get(port_id, &mac);
    RTE_LOG(INFO, KPMD,
            "Port %u MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
            port_id,
            mac.addr_bytes[0], mac.addr_bytes[1], mac.addr_bytes[2],
            mac.addr_bytes[3], mac.addr_bytes[4], mac.addr_bytes[5]);
}

/*  Main poll loop  */
static void
rx_poll_loop(uint16_t port_id, struct rte_mempool *op_pool,
             struct rte_mempool *out_mp, uint16_t bbdev_id)
{
    const uint64_t hz       = rte_get_tsc_hz();
    const uint64_t tsc_end  = rte_rdtsc() + (uint64_t)POLL_SECONDS * hz;
    uint64_t tsc_next_print = rte_rdtsc() + (uint64_t)STATS_PRINT_SEC * hz;

    RTE_LOG(INFO, KPMD,
            "Polling port=%u for %u s (batch_cap=%u)...\n",
            port_id, POLL_SECONDS, NB_BATCH_CAP);

#define TARGET_PACKETS       5000   /* stop after this many test packets */
    while (!force_quit && rte_rdtsc() < tsc_end) {
        if (rx_test >= TARGET_PACKETS) break;

        /*  RX: collect up to NB_BATCH_CAP test packets  */
        struct rte_mbuf *kept[NB_BATCH_CAP];
        uint16_t k = 0;

        while (k < NB_BATCH_CAP && !force_quit) {
            struct rte_mbuf *bufs[BURST_SIZE];
            uint16_t n = rte_eth_rx_burst(port_id, 0, bufs, BURST_SIZE);
            if (n == 0) break;
            rx_total += n;
            for (uint16_t i = 0; i < n && k < NB_BATCH_CAP; i++) {
                if (is_test_packet(bufs[i])) {
                    kept[k++] = bufs[i];
                    rx_test++;
                } else {
                    rte_pktmbuf_free(bufs[i]);
                }
            }
        }

        if (k == 0) goto print_stats;

        /*  Prepare encode ops  */
        struct rte_bbdev_enc_op *ops[NB_BATCH_CAP];
        if (rte_bbdev_enc_op_alloc_bulk(op_pool, ops, k) != 0) {
            op_alloc_drops += k;
            for (uint16_t i = 0; i < k; i++) rte_pktmbuf_free(kept[i]);
            goto print_stats;
        }

        uint16_t prepared = 0;
        for (uint16_t i = 0; i < k; i++) {
            struct rte_mbuf *out_m = rte_pktmbuf_alloc(out_mp);
            if (!out_m) {
                out_alloc_drops++;
                rte_pktmbuf_free(kept[i]);
                rte_bbdev_enc_op_free_bulk(&ops[i], 1);
                continue;
            }

            struct rte_bbdev_enc_op *op = ops[i];

            /* Input */
            uint16_t in_len = (uint16_t)rte_pktmbuf_pkt_len(kept[i]);
            if (in_len > LDPC_INPUT_BYTES) in_len = LDPC_INPUT_BYTES;
            op->ldpc_enc.input.data   = kept[i];
            op->ldpc_enc.input.offset = 0;
            op->ldpc_enc.input.length = in_len;

            /* Output  adj_addr for FECA DMA alignment */
            uint64_t out_bytes = LDPC_OUT_BYTES;
            void *raw_out = rte_pktmbuf_mtod(out_m, void *);
            void *adj_out = rte_pmd_la12xx_ldpc_enc_adj_addr(
                                raw_out, out_bytes);
            op->ldpc_enc.output.data   = out_m;
            op->ldpc_enc.output.offset = adj_out ?
                (uint32_t)((uint8_t *)adj_out - (uint8_t *)raw_out) : 0;
            /* Set exact encoded size  0 may cause QDMA format error */
            op->ldpc_enc.output.length = (uint32_t)out_bytes;

            /* LDPC parameters  Kyocera locked */
            op->ldpc_enc.basegraph       = LDPC_BG;
            op->ldpc_enc.z_c             = LDPC_Z_C;
            op->ldpc_enc.n_cb            = LDPC_N_CB;
            op->ldpc_enc.q_m             = LDPC_Q_M;
            op->ldpc_enc.rv_index        = LDPC_RV;
            op->ldpc_enc.code_block_mode = LDPC_CB_MODE;
            op->ldpc_enc.cb_params.e     = LDPC_E;
            op->ldpc_enc.n_filler        = (in_len < LDPC_INPUT_BYTES) ?
                (uint16_t)(LDPC_K - (in_len * 8)) : 0;
            op->ldpc_enc.op_flags        = 0; /* CRC by LA1224 firmware */

            ops[prepared++] = op;
        }

        /*  Enqueue batch  */
        uint16_t n_enq = 0;
        if (prepared > 0) {
            n_enq = rte_bbdev_enqueue_enc_ops(bbdev_id, QID,
                                              ops, prepared);
            ops_enqueued       += n_enq;
            ops_enqueue_failed += (prepared - n_enq);

            /* Free ops that were not accepted */
            if (n_enq < prepared) {
                for (uint16_t i = n_enq; i < prepared; i++) {
                    rte_pktmbuf_free(ops[i]->ldpc_enc.input.data);
                    rte_pktmbuf_free(ops[i]->ldpc_enc.output.data);
                    rte_bbdev_enc_op_free_bulk(&ops[i], 1);
                }
            }
        }

        /*  Drain batch to completion before next submission  */
        if (n_enq > 0)
            dequeue_batch(bbdev_id, n_enq);

print_stats:
        /*  Periodic stats  */
        {
            uint64_t now = rte_rdtsc();
            if (now >= tsc_next_print) {
                RTE_LOG(INFO, KPMD,
                        "STATS: rx_total=%"PRIu64" rx_test=%"PRIu64"\n"
                        "       ops_enq=%"PRIu64" ops_enq_fail=%"PRIu64"\n"
                        "       ops_deq=%"PRIu64" ops_failed=%"PRIu64"\n"
                        "       tx_sent=%"PRIu64" tx_drop=%"PRIu64"\n"
                        "       op_alloc_drops=%"PRIu64
                        " out_alloc_drops=%"PRIu64"\n",
                        rx_total, rx_test,
                        ops_enqueued, ops_enqueue_failed,
                        ops_dequeued, ops_failed,
                        tx_sent, tx_drop,
                        op_alloc_drops, out_alloc_drops);
                tsc_next_print = now + (uint64_t)STATS_PRINT_SEC * hz;
            }
        }
    }

    RTE_LOG(INFO, KPMD,
            "DONE: rx_total=%"PRIu64" rx_test=%"PRIu64"\n"
            "      ops_enq=%"PRIu64" ops_enq_fail=%"PRIu64"\n"
            "      ops_deq=%"PRIu64" ops_failed=%"PRIu64"\n"
            "      tx_sent=%"PRIu64" tx_drop=%"PRIu64"\n",
            rx_total, rx_test,
            ops_enqueued, ops_enqueue_failed,
            ops_dequeued, ops_failed,
            tx_sent, tx_drop);
}

/*  main  */
int
main(int argc, char **argv)
{
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    int ret = rte_eal_init(argc, argv);
    if (ret < 0)
        rte_panic("EAL init failed: %s\n", rte_strerror(rte_errno));
    argc -= ret; argv += ret;

    enum rte_iova_mode iova = rte_eal_iova_mode();
    RTE_LOG(INFO, KPMD, "IOVA mode: %s\n",
            iova == RTE_IOVA_VA ? "VA (correct for LA12xx)" : "PA (unexpected)");

    /*  NIC mbuf pool  */
    struct rte_mempool *mbuf_pool = rte_pktmbuf_pool_create(
        "KYOCERA_MBUF_POOL", NB_MBUF, MBUF_CACHE_SIZE, 0, MBUF_DATAROOM,
        rte_socket_id());
    if (!mbuf_pool)
        rte_exit(EXIT_FAILURE, "mbuf pool: %s\n", rte_strerror(rte_errno));

    uint16_t port_id;
    RTE_ETH_FOREACH_DEV(port_id) configure_port(port_id, mbuf_pool);
    uint16_t first_port = 0;
    RTE_ETH_FOREACH_DEV(first_port) { break; }
    g_port_id = first_port;

    /*  bbdev init  */
    int bbdev_sock = SOCKET_ID_ANY;
    uint16_t bbdev_id = bbdev_init(&bbdev_sock);

    /*  Map NIC pool into LA12xx window  */
    if (la12xx_map_pool(mbuf_pool, bbdev_id, "KYOCERA_MBUF_POOL") < 0)
        rte_exit(EXIT_FAILURE, "KYOCERA_MBUF_POOL not reachable by LA12xx\n");

    /*  bbdev op pool  */
    struct rte_mempool *op_pool = rte_bbdev_op_pool_create(
        "BBDEV_OP_POOL", RTE_BBDEV_OP_LDPC_ENC,
        BBDEV_OP_POOL_SIZE, BBDEV_OP_POOL_CACHE, bbdev_sock);
    if (!op_pool)
        rte_exit(EXIT_FAILURE, "op pool: %s\n", rte_strerror(rte_errno));
    RTE_LOG(INFO, KPMD, "bbdev op pool ready (size=%u)\n", BBDEV_OP_POOL_SIZE);

    /*  Output pool (encoder output + re-encap TX)  */
    struct rte_mempool *out_mp = rte_pktmbuf_pool_create(
        "OUTPUT_POOL", NB_OUT_MBUFS, 0, 64, OUT_MBUF_SIZE,
        rte_socket_id());
    if (!out_mp)
        rte_exit(EXIT_FAILURE, "out pool: %s\n", rte_strerror(rte_errno));
    g_out_mp = out_mp;
    RTE_LOG(INFO, KPMD,
            "output pool ready (size=%u, buf=%u)\n",
            NB_OUT_MBUFS, OUT_MBUF_SIZE);

    /*  Map output pool into LA12xx window  */
    if (la12xx_map_pool(out_mp, bbdev_id, "OUTPUT_POOL") < 0)
        rte_exit(EXIT_FAILURE, "OUTPUT_POOL not reachable by LA12xx\n");

    /*  Wait for link  */
    RTE_LOG(INFO, KPMD, "Waiting up to 15 s for link UP on port %u...\n",
            first_port);
    for (int i = 0; i < 150; i++) {
        struct rte_eth_link link;
        rte_eth_link_get_nowait(first_port, &link);
        if (link.link_status == RTE_ETH_LINK_UP) {
            RTE_LOG(INFO, KPMD, "Link UP: %u Mbps full-duplex\n",
                    link.link_speed);
            break;
        }
        rte_delay_us(100000);
    }

    RTE_LOG(INFO, KPMD,
            "APP: batch cap=%u (Task 3 NXP QDMA workaround)\n",
            NB_BATCH_CAP);
    RTE_LOG(INFO, KPMD, "APP: waiting on DPMAC.1 RX...\n");

    rx_poll_loop(first_port, op_pool, out_mp, bbdev_id);

    rte_bbdev_stop(bbdev_id);
    rte_bbdev_close(bbdev_id);

    RTE_ETH_FOREACH_DEV(port_id) {
        rte_eth_dev_stop(port_id);
        rte_eth_dev_close(port_id);
    }

    rte_mempool_free(op_pool);
    rte_mempool_free(out_mp);
    rte_mempool_free(mbuf_pool);
    rte_eal_cleanup();
    return 0;
}
