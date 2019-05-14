{ rev    ? "65c3b456b5975787a00708dc9f47b0e03de50207"             # The Git revision of nixpkgs to fetch
, sha256 ? "0qdffvjzz48dpfdh1pmswdvq8ir9pipj9zxysqxkmjj7fhv714gx" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
