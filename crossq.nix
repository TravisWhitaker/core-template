let nativePkgs = import ./default.nix {};
    intelPkgs = import ./default.nix {system = "x86_64-linux";};
in intelPkgs.quartus-prime-lite.override {unstick = nativePkgs.unstick;}
