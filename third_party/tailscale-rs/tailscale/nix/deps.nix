# Unified build of repo Cargo deps. If you need to introduce another library as a
# dependency, you need to include it in buildInputs below. Dependencies that
# run on the build machine should be included in nativeBuildInputs. The
# distinction is important for cross-compilation -- pkgs.callPackage automagically
# decides where to source a package from (pkgsBuildHost, pkgsBuildBuild, etc.)
# based on which *Inputs it's in here.
{
  craneLib,
  openssl,
  pkg-config,
  perl,

  lib,
}: let
  buildDeps = {
    nativeBuildInputs = [
      pkg-config
      perl
    ];

    buildInputs = [
      openssl
    ];
  };

in craneLib.buildDepsOnly (buildDeps // {
  pname = "tailscale-rs-wksp";
  version = "dev";

  strictDeps = true;

  src = lib.fileset.toSource {
    root = ./..;
    fileset = lib.fileset.unions [
      (lib.fileset.fileFilter (file: file.name == "Cargo.toml") ./..)
      (lib.fileset.fileFilter (file: file.name == "Cargo.lock") ./..)
      (lib.fileset.fileFilter (file: file.name == "config.toml") ./..)  # capture .cargo/config.toml
    ];
  };

  passthru = { buildDeps = buildDeps; };
})
