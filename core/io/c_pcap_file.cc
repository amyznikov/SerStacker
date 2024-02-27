/*
 * c_pcap_file.cc
 *
 *  Created on: Mar 11, 2022
 *      Author: amyznikov
 */

#include "c_pcap_file.h"
#include <core/debug.h>

#if HAVE_PCAP

/**
 * Error messages from pcap API
 * */
const char* pcap_errmsg(int status)
{
  switch (status) {
  case PCAP_ERROR:
    return "generic error code";
  case PCAP_ERROR_BREAK:
    return "loop terminated by pcap_breakloop";
  case PCAP_ERROR_NOT_ACTIVATED:
    return "the capture needs to be activated";
  case PCAP_ERROR_ACTIVATED:
    return "the operation can't be performed on already activated captures";
  case PCAP_ERROR_NO_SUCH_DEVICE:
    return "no such device exists";
  case PCAP_ERROR_RFMON_NOTSUP:
    return "this device doesn't support rfmon (monitor) mode";
  case PCAP_ERROR_NOT_RFMON:
    return "operation supported only in monitor mode";
  case PCAP_ERROR_PERM_DENIED:
    return "no permission to open the device";
  case PCAP_ERROR_IFACE_NOT_UP:
    return "interface isn't up";
  case PCAP_ERROR_CANTSET_TSTAMP_TYPE:
    return "this device doesn't support setting the time stamp type";
  case PCAP_ERROR_PROMISC_PERM_DENIED:
    return "you don't have permission to capture in promiscuous mode";
  case PCAP_ERROR_TSTAMP_PRECISION_NOTSUP:
    return "the requested time stamp precision is not supported";
  }
  return "Unknown error code";
}


c_pcap_reader::c_pcap_reader()
{
}

c_pcap_reader::~c_pcap_reader()
{
  close();
}

const std::string & c_pcap_reader::filename() const
{
  return filename_;
}

const std::string & c_pcap_reader::options() const
{
  return options_;
}

uint32_t c_pcap_reader::precision() const
{
  return precision_;
}

int c_pcap_reader::datalinktype() const
{
  return datalinktype_;
}

int c_pcap_reader::data_header_size() const
{
  return data_header_size_;
}


pcap_t * c_pcap_reader::pcap() const
{
  return pcap_;
}

bool c_pcap_reader::open(const std::string & filename, const std::string & filter_arg, uint32_t precision)
{
  close();

  if ( filename.empty() ) {
    CF_ERROR("Not input pcap file name specified");
    return false;
  }

  char errbuf[PCAP_ERRBUF_SIZE];

  constexpr uint32_t loopback_header_size = 4;
  constexpr uint32_t ethernet_header_size = 14;

  bool fOk = false;

  filename_ = filename;
  options_ = filter_arg;
  precision_ = precision;

  // Open pcap file
  if( !(pcap_ = pcap_open_offline_with_tstamp_precision(filename_.c_str(), precision_, errbuf)) ) {
    CF_ERROR("pcap_open_offline_with_tstamp_precision('%s') fails: errno = %d (%s) errbuf='%s'",
        filename.c_str(), errno, strerror(errno), errbuf);
    goto end;
  }

  // Compute filter for the kernel-level filtering engine if requested
  if ( !filter_arg.empty() ) {

    bpf_program filter;

    if( pcap_compile(pcap_, &filter, filter_arg.c_str(), 0, PCAP_NETMASK_UNKNOWN) == -1 ) {
      CF_ERROR("pcap_compile(filter='%s') fails: %s", filter_arg.c_str(), pcap_geterr(pcap_));
      goto end;
    }

    // Associate filter to pcap
    if( pcap_setfilter(pcap_, &filter) < 0 ) {
      CF_ERROR("pcap_setfilter(filter='%s') fails: %s", filter_arg.c_str(), pcap_geterr(pcap_));
      goto end;
    }
  }

  // Determine Datalink header size
  switch ((datalinktype_ = pcap_datalink(pcap_)))
  {
  case DLT_NULL:
    data_header_size_ = sizeof(c_bsd_loopback_header);
    break;
  case DLT_EN10MB:
    data_header_size_ = sizeof(c_en10mb_header);
    break;
  case DLT_LINUX_SLL:
    data_header_size_ = sizeof(struct c_sll_header);
    break;
  case DLT_LINUX_SLL2:
    data_header_size_ = sizeof(struct sll2_header);
    break;
  case DLT_USER1:
    data_header_size_ = sizeof(struct c_dltuser1_header);
    break;
  default:
    data_header_size_ = 0;
    CF_ERROR("Unknown or not supported link type %d in pcap file. "
        "Cannot tell where the payload is.", datalinktype_);
    goto end;
  }

  // finish
  fOk = true;
end:
  if( !fOk ) {
    close();
  }

  return true;
}


bool c_pcap_reader::is_open() const
{
  return pcap_ != nullptr;
}

void c_pcap_reader::close()
{
  if ( pcap_ ) {
    pcap_close(pcap_);
    pcap_ = nullptr;
  }
}


int c_pcap_reader::read(const pcap_pkthdr **pkt_header, const uint8_t ** pkt_data)
{
  int status;

  if ( !pcap_ ) {
    CF_ERROR("Invalid call: pcap file is not open");
    status = PCAP_ERROR_NO_SUCH_DEVICE;
  }
  else if ( (status = pcap_next_ex(pcap_, (pcap_pkthdr **)pkt_header, pkt_data) ) < 0 ) {
    CF_ERROR("pcap_next_ex() fails: status=%d (%s)", status,
        pcap_errmsg(status));
  }

  return status;
}

/** read()
 * Read the next packet from pcap file.
 * The struct pcap_pkthdr and the packet data are not to be freed by the caller,
 * and are not guaranteed to be valid after the next call
 * */
int c_pcap_reader::read(const pcap_pkthdr ** paket_header, const c_pcap_data_header ** data_header, const uint8_t ** payload)
{
  struct pcap_pkthdr * pkt_header = nullptr;
  const uint8_t *pkt_data = nullptr;
  int status;

  if( !pcap_ ) {
    CF_ERROR("Invalid call: pcap file is not open");
    status = PCAP_ERROR_NO_SUCH_DEVICE;
  }
  else if( (status = pcap_next_ex(pcap_, &pkt_header, &pkt_data)) < 0 ) {
    CF_ERROR("pcap_next_ex() fails: status=%d (%s)", status,
        pcap_errmsg(status));
  }
  else if( status > 0 ) {

    if ( paket_header ) {
      *paket_header = pkt_header;
    }

    if( data_header ) {
      *data_header = (const c_pcap_data_header*) pkt_data;
    }

    // See <https://www.ietf.org/id/draft-ietf-opsawg-pcap-00.html#linktype>
    switch (datalinktype_) {

    case DLT_NULL:
      if( pkt_header->len < sizeof(c_bsd_loopback_header) ) {
        CF_ERROR("Invalid pkt_header->len=%u < loopback_header size=%zu",
            pkt_header->len, sizeof(c_bsd_loopback_header));
        status = PCAP_ERROR;
      }
      else if( payload ) {
        *payload = pkt_data + sizeof(c_bsd_loopback_header);
      }
      break;

    case DLT_EN10MB:
      if( pkt_header->len < sizeof(c_en10mb_header) ) {
        CF_ERROR("Invalid pkt_header->len=%u < ethernet_header size=%zu",
            pkt_header->len, sizeof(c_en10mb_header));
        status = PCAP_ERROR;
      }
      else if( payload ) {
        *payload = pkt_data + sizeof(c_en10mb_header);
      }
      break;

    case DLT_LINUX_SLL:
      if( pkt_header->len < sizeof(struct c_sll_header) ) {
        CF_ERROR("Invalid pkt_header->len=%u < c_sll_header size=%zu",
            pkt_header->len, sizeof(struct c_sll_header));
        status = PCAP_ERROR;
      }
      else if( payload ) {
        *payload = pkt_data + sizeof(struct c_sll_header);
      }
      break;

    case DLT_LINUX_SLL2:
      if( pkt_header->len < sizeof(struct c_sll2_header) ) {
        CF_ERROR("Invalid pkt_header->len=%u < c_sll2_header=%zu",
            pkt_header->len, sizeof(struct c_sll2_header));
        status = PCAP_ERROR;
      }
      else if( payload ) {
        *payload = pkt_data + sizeof(struct c_sll2_header);
      }
      break;

    case DLT_USER1:
      if( pkt_header->len < sizeof(struct c_dltuser1_header) ) {
        // CF_ERROR("Invalid pkt_header->len=%u < sizeof(c_dltuser1_header)=%zu",
        //  pkt_header->len, sizeof(c_dltuser1_header));
        status = 0;
        if( payload ) {
          *payload = nullptr;
        }
      }
      else if( payload ) {
        *payload = pkt_data +
            sizeof(struct c_dltuser1_header);
      }
      break;

    default:
      CF_ERROR("Unknown or not supported link type %d in pcap file. "
          "Cannot tell where the payload is.", datalinktype_);
      status = PCAP_ERROR;
      break;
    }
  }

  return status;
}


/** Set current read file position in bytes*/
long c_pcap_reader::seek(long position, int whence)
{
  if ( !pcap_ ) {
    CF_ERROR("Invalid call: pcap file is not open");
    return -1;
  }

  if( fseek(pcap_file(pcap_), position, whence) < 0 ) {
    CF_ERROR("fseek(pcap_file) fails: errno=%d (%s)", errno, strerror(errno));
  }

  return ftell(pcap_file(pcap_));
}

/** Get current read file position in bytes*/
long c_pcap_reader::tell() const
{
  if ( !pcap_ ) {
    CF_ERROR("Invalid call: pcap file is not open");
    return -1;
  }

  return ftell(pcap_file(pcap_));
}

#endif // HAVE_PCAP
