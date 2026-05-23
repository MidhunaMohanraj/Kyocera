/*
 * kyocera_pmd_init — Week 4 / Task 3 skeleton
 *
 * DPDK EAL init + DPAA2 port config + pktmbuf mempool + RX/TX queue setup,
 * followed by a 30-second RX poll loop to prove the datapath is live.
 *
 * Target: NXP LX2160A, NXP LSDK 2004 DPDK 19.11-qoriq, BSP 3.2.
 * Prereq: hugepages (1 GiB) reserved, DPRC=dprc.2 bound to vfio-fsl-mc.
 */  
#include <stdio.h>        
#include <stdint.h>   
#include <string.h>    
#include <inttypes.h>      
#include <signal.h>  
#include <stdlib.h>     
         
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_mbuf.h>         
#include <rte_mempool.h>
#include <rte_lcore.h>
#include <rte_cycles.h>
#include <rte_debug.h>
#include <rte_errno.h>
#include <rte_log.h>

#define NB_MBUF           8192
#define MBUF_CACHE_SIZE   256
#define MBUF_DATAROOM     RTE_MBUF_DEFAULT_BUF_SIZE

#define NB_RX_QUEUES      1
#define NB_TX_QUEUES      1
#define NB_RX_DESC        1024
#define NB_TX_DESC        1024

#define BURST_SIZE        32
#define POLL_SECONDS      30

#define RTE_LOGTYPE_KPMD  RTE_LOGTYPE_USER1

static volatile sig_atomic_t force_quit;

static void
handle_signal(int sig)
{
    (void)sig;
    force_quit = 1;
}

static struct rte_eth_conf port_conf_default = {
    .rxmode = {
        .mq_mode        = ETH_MQ_RX_NONE,
        .max_rx_pkt_len = RTE_ETHER_MAX_LEN,
    },
    .txmode = {
        .mq_mode = ETH_MQ_TX_NONE,
    },
};

static void
log_port_info(uint16_t port_id, const struct rte_eth_dev_info *info)
{
    RTE_LOG(INFO, KPMD,
            "Port %u: driver=%s max_rxq=%u max_txq=%u "
            "rx_offload_capa=0x%" PRIx64 " tx_offload_capa=0x%" PRIx64 "\n",
            port_id, info->driver_name,
            info->max_rx_queues, info->max_tx_queues,
            info->rx_offload_capa, info->tx_offload_capa);
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
            link.link_duplex == ETH_LINK_FULL_DUPLEX ? "full" : "half");
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
        rte_exit(EXIT_FAILURE, "dev_configure(port=%u): %s (%d)\n",
                 port_id, rte_strerror(-ret), ret);

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
            rte_exit(EXIT_FAILURE,
                     "rx_queue_setup(port=%u q=%u): %d\n", port_id, q, ret);
    }

    struct rte_eth_txconf txconf = dev_info.default_txconf;
    txconf.offloads = port_conf.txmode.offloads;
    for (uint16_t q = 0; q < NB_TX_QUEUES; q++) {
        ret = rte_eth_tx_queue_setup(port_id, q, nb_txd, sock, &txconf);
        if (ret < 0)
            rte_exit(EXIT_FAILURE,
                     "tx_queue_setup(port=%u q=%u): %d\n", port_id, q, ret);
    }

    ret = rte_eth_dev_start(port_id);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "dev_start(port=%u): %d\n", port_id, ret);

    rte_eth_promiscuous_enable(port_id);

    log_mac_and_link(port_id);
    return 0;
}

static void
rx_poll_loop(uint16_t port_id)
{
    struct rte_eth_stats s0;
    rte_eth_stats_get(port_id, &s0);

    uint64_t rx_total = 0;
    const uint64_t tsc_end = rte_rdtsc() + (uint64_t)POLL_SECONDS * rte_get_tsc_hz();

    RTE_LOG(INFO, KPMD,
            "Polling port=%u q=0 for %u s on lcore %u (Ctrl-C to exit early)...\n",
            port_id, POLL_SECONDS, rte_lcore_id());

    while (!force_quit && rte_rdtsc() < tsc_end) {
        struct rte_mbuf *bufs[BURST_SIZE];
        uint16_t n = rte_eth_rx_burst(port_id, 0, bufs, BURST_SIZE);
        if (n == 0) continue;

        rx_total += n;
        for (uint16_t i = 0; i < n; i++)
            rte_pktmbuf_free(bufs[i]);
    }

    struct rte_eth_stats s1;
    rte_eth_stats_get(port_id, &s1);

    RTE_LOG(INFO, KPMD,
            "Done. RX mbufs=%" PRIu64
            " | ipackets delta=%" PRIu64
            " ibytes delta=%" PRIu64
            " imissed delta=%" PRIu64
            " ierrors delta=%" PRIu64 "\n",
            rx_total,
            s1.ipackets - s0.ipackets,
            s1.ibytes   - s0.ibytes,
            s1.imissed  - s0.imissed,
            s1.ierrors  - s0.ierrors);
}

int
main(int argc, char **argv)
{
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    int ret = rte_eal_init(argc, argv);
    if (ret < 0)
        rte_panic("EAL init failed: %s (errno=%d)\n",
                  rte_strerror(rte_errno), rte_errno);
    argc -= ret;
    argv += ret;

    uint16_t nb_ports = rte_eth_dev_count_avail();
    if (nb_ports == 0)
        rte_exit(EXIT_FAILURE,
                 "No Ethernet ports. Is DPRC set and vfio-fsl-mc bound?\n");
    RTE_LOG(INFO, KPMD, "Found %u Ethernet port(s).\n", nb_ports);

    struct rte_mempool *mp = rte_pktmbuf_pool_create(
        "KYOCERA_MBUF_POOL",
        NB_MBUF, MBUF_CACHE_SIZE,
        0, MBUF_DATAROOM,
        rte_socket_id());
    if (mp == NULL)
        rte_exit(EXIT_FAILURE, "pktmbuf_pool_create: %s\n",
                 rte_strerror(rte_errno));

    uint16_t port_id;
    RTE_ETH_FOREACH_DEV(port_id) {
        configure_port(port_id, mp);
    }

    uint16_t first_port = 0;
    RTE_ETH_FOREACH_DEV(first_port) { break; }

    rx_poll_loop(first_port);

    RTE_ETH_FOREACH_DEV(port_id) {
        rte_eth_dev_stop(port_id);
        rte_eth_dev_close(port_id);
    }
    rte_mempool_free(mp);
    rte_eal_cleanup();
    return 0;
}
