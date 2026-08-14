# GitHub Pages redirect

This branch is the GitHub Pages source for `xronos-inc.github.io/xronos`. It
exists only to redirect visitors to the canonical documentation at
[docs.xronos.com](https://docs.xronos.com/).

The documentation used to be built into `docs/` on `main` and deployed by a
workflow. It now lives at docs.xronos.com, and neither the sources nor the
workflow are in this repository anymore.

`index.html` catches the site root. `404.html` catches every other path, since
none of the old pages exist here. Both redirect to docs.xronos.com and preserve
the request path, so old deep links keep working. GitHub Pages cannot serve real
301 responses, so they use a meta refresh and a canonical link instead.

Pages must stay configured to deploy from this branch. Release automation
overwrites `main`, so redirect files kept there would not survive.
