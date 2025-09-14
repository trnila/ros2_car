bazel build //... --platforms=//:aarch64 && rsync -Lavz bazel-bin/ car:/tmp/build/
