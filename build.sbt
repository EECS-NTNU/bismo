val chiselVersion = System.getProperty("chiselVersion", "2.+")
val scalaVer = System.getProperty("scalaVer", "2.11.6")

lazy val bismoSettings = Seq (
  name := "bismo",
  version := "0.1",
  scalaVersion := scalaVer,
  libraryDependencies ++= ( if (chiselVersion != "None" ) ("edu.berkeley.cs" %% "chisel" % chiselVersion) :: Nil; else Nil),
  libraryDependencies += "com.novocode" % "junit-interface" % "0.10" % "test",
  libraryDependencies += "org.scalatest" %% "scalatest" % "2.2.4" % "test",
  libraryDependencies += "org.scala-lang" % "scala-compiler" % scalaVer
)

// add fpga-tidbits as unmanaged source dependency, pulled as git submodule
unmanagedSourceDirectories in Compile += baseDirectory.value / "fpga-tidbits" / "src" / "main" / "scala"
// fpga-tidbits stores compile scripts, drivers etc. in the resource dir
unmanagedResourceDirectories in Compile += baseDirectory.value / "fpga-tidbits" / "src" / "main" / "resources"

lazy val bismo_template = (project in file(".")).settings(bismoSettings: _*)
