{ rev    ? "dec69e24337cbf22f602b290a0907af54dc06556"             # The Git revision of nixpkgs to fetch
, sha256 ? "09ckkk6y7rwijdlvhd7d9igf77byvda6j2zcvnaad5a1ybds2p89" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
