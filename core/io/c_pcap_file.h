/*
 * c_pcap_file.h
 *
 *  Created on: Mar 11, 2022
 *      Author: amyznikov
*
 * For pcap link types see
 *  <https://www.ietf.org/id/draft-ietf-opsawg-pcap-00.html>
 *
 * For capturing lidar data use tcpdump
 *  tcpdump -vv -i any -u "src 192.168.2.201 and udp and port 2368" -w test.pcap
 *
 */

#ifndef __c_pcap_file_h__
#define __c_pcap_file_h__

// #define HAVE_PCAP 1
#if HAVE_PCAP // Must be wet from CMakeList.txt

#include <string>
#include <inttypes.h>
#include <pcap/pcap.h>
#include <pcap/dlt.h>
#include <pcap/sll.h>


#if !_WIN32
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#else

/**
 * This stuff is copied from linux headers to support MSYS2 build
 */

/*
 *  IEEE 802.3 Ethernet magic constants.  The frame sizes omit the preamble
 *  and FCS/CRC (frame check sequence).
 */

#define ETH_ALEN  6   /* Octets in one ethernet addr   */
#define ETH_TLEN  2   /* Octets in ethernet type field */
#define ETH_HLEN  14    /* Total octets in header.   */
#define ETH_ZLEN  60    /* Min. octets in frame sans FCS */
#define ETH_DATA_LEN  1500    /* Max. octets in payload  */
#define ETH_FRAME_LEN 1514    /* Max. octets in frame sans FCS */
#define ETH_FCS_LEN 4   /* Octets in the FCS     */

#define ETH_MIN_MTU 68    /* Min IPv4 MTU per RFC791  */
#define ETH_MAX_MTU 0xFFFFU   /* 65535, same as IP_MAX_MTU  */

#pragma pack(push,1)

struct ether_addr
{
  uint8_t ether_addr_octet[ETH_ALEN];
};

/* 10Mb/s ethernet header */
struct ether_header
{
  uint8_t  ether_dhost[ETH_ALEN]; /* destination eth addr */
  uint8_t  ether_shost[ETH_ALEN]; /* source ether addr  */
  uint16_t ether_type;            /* packet type ID field */
};


/*
 * Definitions for internet protocol version 4.
 * Per RFC 791, September 1981.
 */
#define IPVERSION 4

/*
 * Structure of an internet header, naked of options.
 */
struct ip {
#ifdef _IP_VHL
  uint8_t  ip_vhl;   /* version << 4 | header length >> 2 */
#else
#if __BYTE_ORDER == __LITTLE_ENDIAN
  uint8_t ip_hl:4,   /* header length */
         ip_v:4;    /* version */
#elif __BYTE_ORDER == __BIG_ENDIAN
  uint8_t ip_v:4,    /* version */
         ip_hl:4;   /* header length */
#endif
#endif /* not _IP_VHL */
  uint8_t  ip_tos;    /* type of service */
  uint16_t ip_len;   /* total length */
  uint16_t ip_id;    /* identification */
  uint16_t ip_off;   /* fragment offset field */
#define IP_RF 0x8000      /* reserved fragment flag */
#define IP_DF 0x4000      /* dont fragment flag */
#define IP_MF 0x2000      /* more fragments flag */
#define IP_OFFMASK 0x1fff   /* mask for fragmenting bits */
  uint8_t  ip_ttl;   /* time to live */
  uint8_t  ip_p;     /* protocol */
  uint16_t ip_sum;   /* checksum */
  struct  in_addr ip_src,ip_dst;  /* source and dest address */
};

#define UDP_SEGMENT  2  /* WinSock UDP_SEND_MSG_SIZE */
#define UDP_GRO    3  /* WinSock UDP_RECV_MAX_COALESCED_SIZE,
           also == UDP_COALESCED_INFO */

/*
 * Udp protocol header.
 * Per RFC 768, September, 1981.
 */
struct udphdr {
  uint16_t uh_sport;   /* source port */
  uint16_t uh_dport;   /* destination port */
  uint16_t uh_ulen;    /* udp length */
  uint16_t uh_sum;   /* udp checksum */
};


#pragma pack(pop)

#endif


#pragma pack(push,1)


/**
 * c_bsd_loopback_header
 * DLT_NULL
 *
 * BSD loopback encapsulation header
 **/
struct c_bsd_loopback_header
{
  uint32_t protocol; // AF_INET etc
  struct ip ip;
  struct udphdr udp;
};

/**
 * c_en10mb_header
 * DLT_EN10MB
 *
 *  10Mb/s ethernet header + ip + udp
 *  48 bytes
 * */
struct c_en10mb_header
{
  struct ether_header eth;
  struct ip ip;
  struct udphdr udp;
};

/**
 * c_sll_header
 * DLT_LINUX_SLL
 * Linux cooked capture encapsulation
 */
struct c_sll_header
{
  struct sll_header sll;
  struct ip ip;
  struct udphdr udp;
};

/**
 * c_sll2_header
 * DLT_LINUX_SLL2
 * Linux cooked sockets v2
 */
struct c_sll2_header
{
  struct sll2_header sll2;
  struct ip ip;
  struct udphdr udp;
};

/**
 * c_dltuser1_header
 * DLT_USER1 = 148
 * Reserved for private use.
 * Probably zenuity-specific DLT
 *
 * 69 106 8 0 VS128
 * 56 184 8 0 VLP16 1
 * 56 198 8 0 VLP16 2
 */

enum c_zlidar_id
{
  c_zlidar_id_any = -1,
  c_zlidar_id_vlp16_1 = (56 + (184<<8) + (8 << 16)),
  c_zlidar_id_vlp16_2 = (56 + (198<<8) + (8 << 16)),
  c_zlidar_id_vls128 = (69 + (106<<8) + (8 << 16)),
};

struct c_dltuser1_header
{
  uint8_t unknown_bytes[54 - sizeof(struct ip) - sizeof(struct udphdr) - 4];
  uint32_t zlidarid;
  struct ip ip;
  struct udphdr udp;
};



/**
 * c_pcap_data_header
 * */
union c_pcap_data_header
{
  struct c_en10mb_header en10mb;
  struct c_bsd_loopback_header loopbak;
  struct c_sll_header sll;
  struct c_sll2_header sll2;
  struct c_dltuser1_header user1;
};


#pragma pack(pop)


/**
 * c_pcap_reader
 * Class-wraper around of pcap reading functions
 * */
class c_pcap_reader
{
public:

  /* Default c'tor */
  c_pcap_reader();

  /* Default d'tor, close file on exit */
  ~c_pcap_reader();

  /** Get current pcap file name */
  const std::string & filename() const;

  /** Get current pcap options */
  const std::string & options() const;

  /* Access to pcap file handle */
  pcap_t * pcap() const;

  /* Get time stamp precison, one of
   * PCAP_TSTAMP_PRECISION_MICRO or PCAP_TSTAMP_PRECISION_NANO
   * */
  uint32_t precision() const;

  int data_header_size() const;

  int datalinktype() const;

  /* Open pcap file for read */
  bool open(const std::string & filename, const std::string & filter = "",
      uint32_t precision = PCAP_TSTAMP_PRECISION_MICRO);

  /* Check if file is open */
  bool is_open() const;

  void close();

  /** read()
   * Read the next packet from pcap file.
   * The struct pcap_pkthdr and the packet data are not to be freed by the caller,
   * and are not guaranteed to be valid after the next call */
  int read(const pcap_pkthdr **pkt_header,
      const uint8_t **pkt_data);

  /** read()
   * Read the next packet from pcap file.
   * The struct pcap_pkthdr and the packet data are not to be freed by the caller,
   * and are not guaranteed to be valid after the next call */
  int read(const pcap_pkthdr ** pkt_header,
      const c_pcap_data_header ** data_header,
      const uint8_t ** payload);

  /** Set current read file position in bytes*/
  long seek(long position, int whence = SEEK_SET);

  /** Get current read file position in bytes*/
  long tell() const;

protected:
  pcap_t * pcap_ = nullptr;
  std::string filename_;
  std::string options_;
  uint32_t precision_ = PCAP_TSTAMP_PRECISION_MICRO;
  int datalinktype_ = 0;
  int data_header_size_ = -1;
};

/**
 * Error messages from pcap API
 * */
const char* pcap_errmsg(int status);

#endif // HAVE_PCAP

#endif /* __c_pcap_file_h__ */
