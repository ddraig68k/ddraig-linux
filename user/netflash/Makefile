-include ../user.mak

CFLAGS += -Wall
CFLAGS += -D_GNU_SOURCE

EXEC = netflash
EXTRA_TARGETS :=
OBJS = tftpmain.o tftp.o tftpsubs.o http.o fileblock.o fileblock_ram.o fileblock_file.o base64.o program.o check.o
COBJS = netflash.o

OBJS += util.o

ifdef CONFIG_USER_NETFLASH_WITH_FTP
CPPLAGS += -DFTP
OBJS += cmds.o cmdtab.o domacro.o ftp.o glob.o ftpmain.o ruserpass.o
endif

ifdef CONFIG_USER_NETFLASH_WITH_CGI
OBJS += cgi.o cgiparse.o kmp.o
endif

ifdef CONFIG_USER_NETFLASH_HMACMD5
EXEC += hmacmd5
OBJS += md5.o hmacmd5.o
endif

ifdef HW_COMPAT_VENDOR
VENDOR=$(HW_COMPAT_VENDOR)
else
ifdef HW_VENDOR
VENDOR=$(HW_VENDOR)
else
VENDOR=$(CONFIG_VENDOR)
endif
endif
CPPFLAGS += -DVENDOR=\"$(VENDOR)\"

ifdef HW_COMPAT_PRODUCT
PRODUCT=$(HW_COMPAT_PRODUCT)
else
ifdef HW_PRODUCT
PRODUCT=$(HW_PRODUCT)
else
PRODUCT=$(CONFIG_PRODUCT)
endif
endif
CPPFLAGS += -DPRODUCT=\"$(PRODUCT)\"

CPPFLAGS += -DVERSION=\"$(VERSIONPKG)\"
COBJS += versioning.o

ifdef CONFIG_USER_NETFLASH_DECOMPRESS
OBJS += decompress.o
LIBS = -lz
endif

ifdef CONFIG_USER_NETFLASH_SETSRC
CPPFLAGS += -DSETSRC
endif

ifdef CONFIG_USER_NETFLASH_CRYPTO

ifdef HOSTBUILD
PUBLIC_KEY_FILE=netflash.pem
else
ifdef CONFIG_USER_NETFLASH_CRYPTO_V2
PUBLIC_KEY_FILE=/etc/publickey.pem
else
ifdef CONFIG_USER_FLATFSD_ETC_CONFIG
PUBLIC_KEY_FILE=/etc/config/netflash.pem
else
PUBLIC_KEY_FILE=/etc/netflash.pem
endif
endif
endif
CPPFLAGS += -DPUBLIC_KEY_FILE=\"$(PUBLIC_KEY_FILE)\"

LIBS += $(LIBCRYPTO)
OBJS += crypto.o
EXTRA_TARGETS += flashkey cryptimage decryptimage
endif

ifdef CONFIG_USER_NETFLASH_CRYPTO_V3
LIBS += $(LIBCRYPTO)
OBJS += cryptov3.o
endif

ifdef CONFIG_USER_NETFLASH_SHA256
LIBS += $(LIBCRYPTO)
EXEC += sha256sum
endif

CPPFLAGS += -Dnetflashmain=main
# CPPFLAGS += -DDEBUG_CGI


all: $(EXEC) $(EXTRA_TARGETS)

netflash: $(OBJS) build_netflash build_versioning
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(COBJS) $(LIBS) $(LDLIBS$(LDLIBS_$@))

build_netflash:
	$(CC) -c -o netflash.o netflash.c $(CFLAGS) $(CPPFLAGS)

build_versioning:
	$(CC) -c -o versioning.o versioning.c $(CFLAGS) $(CPPFLAGS)

hmacmd5: hmacmd5main.c hmacmd5.c md5.c
	$(HOSTCC) -o $@ $^

sha256sum: sha256sum.c sha256.c
	$(HOSTCC) -o $@ $^

versiontest: versioning.c
	$(HOSTCC) -DVERSIONTEST -o $@ $^

flashkey.o cryptimage decryptimage: crypto.h

cryptimage: cryptimage.c

decryptimage: decryptimage.c

cryptimage decryptimage:
	$(HOSTCC) -O2 $(CPPFLAGS) $(HOSTCFLAGS) -I$(STAGEDIR)/include -o $@ $@.c -lssl -lcrypto

flashkey: flashkey.o
	$(CC) $(LDFLAGS) -o $@ flashkey.o $(LIBCRYPTO) $(LDLIBS$(LDLIBS_$@))

romfs:
	$(ROMFSINST) /bin/netflash
ifdef CONFIG_USER_NETFLASH_CRYPTO
	$(ROMFSINST) /bin/flashkey
endif

clean:
	-rm -f versiontest netflash hmacmd5 sha256sum *.elf *.gdb *.o $(EXTRA_TARGETS)

test:
	chmod +x test.tcl
	./test.tcl
