cmd_../linux_headers//include/drm/.install := /bin/bash scripts/headers_install.sh ../linux_headers//include/drm ./include/uapi/drm drm.h drm_fourcc.h drm_mode.h drm_sarea.h exynos_drm.h i810_drm.h i915_drm.h mga_drm.h msm_drm.h nouveau_drm.h qxl_drm.h r128_drm.h radeon_drm.h savage_drm.h sis_drm.h tegra_drm.h via_drm.h vmwgfx_drm.h; /bin/bash scripts/headers_install.sh ../linux_headers//include/drm ./include/drm ; /bin/bash scripts/headers_install.sh ../linux_headers//include/drm ./include/generated/uapi/drm ; for F in ; do echo "\#include <asm-generic/$$F>" > ../linux_headers//include/drm/$$F; done; touch ../linux_headers//include/drm/.install
