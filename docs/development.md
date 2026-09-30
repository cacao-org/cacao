# Development

## Building and testing changes

cacao builds inside a milk source tree, so to build local changes, make your
cacao checkout milk's `plugins/cacao-src`. A symlink works, because milk
follows symlinks when it looks for plugins:

```bash
cd milk
ln -s /path/to/your/cacao plugins/cacao-src
cmake -S . -B _build -DCMAKE_BUILD_TYPE=Release
cmake --build _build -j "$(nproc)"
```

Or build in Docker. The `Dockerfile` clones milk and copies in the current
cacao working tree, so uncommitted changes are included:

```bash
docker build -t cacao .
```

## Continuous integration

GitHub Actions runs three workflows:

- `cmake.yml`: checks out milk (`dev` branch), checks out the pushed cacao
  commit into `milk/plugins/cacao-src`, and builds.
- `docker-image.yml`: builds the `Dockerfile`.
- `docs.yml`: builds this documentation with `tox -e docs`, treating warnings
  as errors. On pushes to `dev`, it also publishes the documentation to
  GitHub Pages at <https://caltechopticalobservatories.github.io/cacao/>.

## Documentation

The documentation is written in Markdown (MyST) under `docs/`, and built with
Sphinx:

```bash
pip install tox
tox -e docs            # HTML output in docs/_build/html
tox -e docs-linkcheck  # check external links
```

The build fails on any Sphinx warning, including broken cross-references.

Older documentation from cacao 1.x, written for the retired `aolconf`
interface, is still in `AOloopControl/doc/`. It hasn't been reviewed yet, so it
isn't part of this site.
