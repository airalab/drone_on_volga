{ stdenv
, python3Packages
, mkRosPackage
, robonomics_comm 
, robonomics_comm-nightly
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "drone_on_volga";
  version = "master";

  src = ./.;

  propagatedBuildInputs = [
    robonomics_comm-nightly
  ];

  meta = with stdenv.lib; {
    description = "Drone on Volga package";
    homepage = http://github.com/airalab/drone_on_volga;
    license = licenses.bsd3;
    maintainers = [ "Alisher Khassanov <alisher@aira.life>" ];
  };
}
