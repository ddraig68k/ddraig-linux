URL = http://www.tcpdump.org/release/tcpdump-4.6.2.tar.gz

CONFVARS :=
CONFVARS += PCAP_CONFIG=$(ROOTDIR)/tools/pcap-config
CONFVARS += ac_cv_linux_vers=2
CONFVARS += td_cv_buggygetaddrinfo=false

CONFOPTS = --without-crypto
CONFOPTS += --disable-smb
ifdef CONFIG_IPV6
CONFOPTS += --enable-ipv6
endif
ifdef CONFIG_LIB_LIBNL_GENERIC
LDFLAGS += -lnl
endif

include $(ROOTDIR)/tools/automake.inc

