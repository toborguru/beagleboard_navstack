#############################################################
#
# uisp
#
#############################################################
UISP_SOURCE=uisp-20050207.tar.gz
UISP_SITE=http://savannah.nongnu.org/download/uisp
UISP_DIR=$(BUILD_DIR)/uisp-20050207
UISP_BINARY=uisp
UISP_TARGET_BINARY=usr/bin/uisp

$(DL_DIR)/$(UISP_SOURCE):
	$(WGET) -P $(DL_DIR) $(UISP_SITE)/$(UISP_SOURCE)

$(UISP_DIR)/.source: $(DL_DIR)/$(UISP_SOURCE)
	zcat $(DL_DIR)/$(UISP_SOURCE) | tar -C $(BUILD_DIR) -xf -
	$(SOURCE_DIR)/patch-kernel.sh $(UISP_DIR) $(SOURCE_DIR) uisp*.patch
	touch $(UISP_DIR)/.source

$(UISP_DIR)/.configured: $(UISP_DIR)/.source
	(cd $(UISP_DIR); \
		$(TARGET_CONFIGURE_OPTS) \
		CFLAGS="$(TARGET_CFLAGS)" \
		ac_cv_func_malloc_0_nonnull=yes \
		./configure \
		--target=$(GNU_TARGET_NAME) \
		--host=$(GNU_TARGET_NAME) \
		--build=$(GNU_HOST_NAME) \
		--prefix=/usr \
		--program-prefix= \
		--sysconfdir=/etc \
	);
	touch $(UISP_DIR)/.configured;

$(UISP_DIR)/$(UISP_BINARY): $(UISP_DIR)/.configured
	$(MAKE) CC=$(TARGET_CC) -C $(UISP_DIR)

$(TARGET_DIR)/$(UISP_TARGET_BINARY): $(UISP_DIR)/$(UISP_BINARY)
	$(MAKE) prefix=$(TARGET_DIR)/usr \
	    sysconfdir=$(TARGET_DIR)/etc \
		-C $(UISP_DIR) install-exec
	rm -Rf $(TARGET_DIR)/usr/man

uisp: uclibc $(TARGET_DIR)/$(UISP_TARGET_BINARY)

uisp-source: $(DL_DIR)/$(UISP_SOURCE)

uisp-clean:
	$(MAKE) prefix=$(TARGET_DIR)/usr -C $(UISP_DIR) uninstall
	-$(MAKE) -C $(UISP_DIR) clean

uisp-dirclean:
	rm -rf $(UISP_DIR)

