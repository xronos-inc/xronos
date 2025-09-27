## Building from Source

We use [docker bake](https://docs.docker.com/build/bake/) as our top-level build
tool. To build locally, install [docker](https://docker.com) and the [buildx
extension](https://github.com/docker/buildx) (if not included in your docker
installation). Then run:

```bash
docker buildx bake build
```
