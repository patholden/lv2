cmd_../linux_headers//include/linux/nfsd/.install := /bin/bash scripts/headers_install.sh ../linux_headers//include/linux/nfsd ./include/uapi/linux/nfsd cld.h debug.h export.h nfsfh.h stats.h; /bin/bash scripts/headers_install.sh ../linux_headers//include/linux/nfsd ./include/linux/nfsd ; /bin/bash scripts/headers_install.sh ../linux_headers//include/linux/nfsd ./include/generated/uapi/linux/nfsd ; for F in ; do echo "\#include <asm-generic/$$F>" > ../linux_headers//include/linux/nfsd/$$F; done; touch ../linux_headers//include/linux/nfsd/.install
