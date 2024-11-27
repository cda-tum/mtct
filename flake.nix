{
  description = "MTCT DevShell";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.05";
  };

  outputs = inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
        "x86_64-darwin"
      ];
      perSystem = { config, self', inputs', pkgs, system, ... }: {
        devShells.default =
          let
            # Need to cache default package first if download is unavailable
            gurobi-with-source = pkgs.gurobi.overrideAttrs (finalAttrs: previousAttrs: {
              installPhase = previousAttrs.installPhase + ''
                mkdir -p $out/src
                cp src/* $out/src/ -r
              '';
            });
          in
          pkgs.mkShell {
            packages = with pkgs; [
              # C++ Compiler is already part of stdenv
              cmake
              lldb
              python3
              pre-commit
              clang-tools
              graphia
            ];

            GUROBI_HOME = "${gurobi-with-source}";
          };
      };
    };
}
