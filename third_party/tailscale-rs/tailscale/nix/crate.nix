# Build a tailscale-rs workspace crate by its Cargo.toml.
{
  craneLib,
  lib,

  deps,
  cargoToml,
  rustsrc,
}: let
  meta = craneLib.crateNameFromCargoToml { inherit cargoToml; };

  buildPackage = {
    passthru ? {},
    cargo_args ? "",
    suffix ? null,
  }: craneLib.buildPackage (deps.passthru.buildDeps // {
    pname = meta.pname;
    version = meta.version;

    strictDeps = true;
    src = rustsrc;

    cargoArtifacts = deps;
    cargoExtraArgs = "-p ${meta.pname} ${cargo_args}";

    passthru = passthru;
  } // lib.optionalAttrs (suffix != null) {
    pnameSuffix = suffix;
  });

in buildPackage {
  passthru = {
    deps = deps;
    examples = buildPackage {
      suffix = "-examples";
      cargo_args = "--examples";
    };
  };
}
