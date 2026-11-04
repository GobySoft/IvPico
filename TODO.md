# TODO

- Recombine the `moos-ivp-helm` packages into the `moos-ivp` project in
  [openocean-build](https://github.com/openocean-software/openocean-build)
  once a MOOS-IvP release includes moos-ivp/moos-ivp#114 and #147. Until
  then `libivp-helm0` and `libivp-helm-dev` are continuous snapshots of
  GobySoft/moos-ivp's `ivp-without-moos` branch, in their own prefix
  (`/usr/lib/<multiarch>/ivp-helm`) so they coexist with the released
  `libmoos-ivp` packages.
- Point `external/moos-ivp` at the signed head of `ivp-without-moos`
  (`4ad4ef9`); `06f25ec` was replaced when it was signed.
