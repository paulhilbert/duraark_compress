#!/bin/sh

set -e

/bin/sh /tmp/build.sh jbig2enc
/bin/sh /tmp/build.sh jbig2dec
/bin/sh /tmp/build.sh pcl_compress
