cmd_../linux_headers//include/misc/.install := /bin/bash scripts/headers_install.sh ../linux_headers//include/misc ./include/uapi/misc cxl.h; /bin/bash scripts/headers_install.sh ../linux_headers//include/misc ./include/misc ; /bin/bash scripts/headers_install.sh ../linux_headers//include/misc ./include/generated/uapi/misc ; for F in ; do echo "\#include <asm-generic/$$F>" > ../linux_headers//include/misc/$$F; done; touch ../linux_headers//include/misc/.install
