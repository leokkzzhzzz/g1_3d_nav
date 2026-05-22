#!/bin/bash
mkdir -p /usr/local/lib/cmake /usr/local/include
ln -sf /root/g1_deps/open3d141/lib/cmake/Open3D /usr/local/lib/cmake/Open3D
rm -rf /usr/local/include/open3d
ln -sf /root/g1_deps/open3d141/include/open3d /usr/local/include/open3d
for f in /root/g1_deps/open3d141/lib/*.a /root/g1_deps/open3d141/lib/*.so*; do
    ln -sf "$f" /usr/local/lib/$(basename "$f") 2>/dev/null || true
done
echo "[entrypoint] Open3D linked"
exec "$@"
