from conans import ConanFile, CMake, tools
from conans.errors import ConanInvalidConfiguration
import os
import textwrap
# Turns out there is no Pangolin package by default on the conan center. 
# TODO: Create conan package of Pangolin
required_conan_version = ">=1.43.0"

class InverseKinematics(ConanFile):

    name= "InverseKinematics"
    license = "MIT"
    description = "Inverse Kinematics, Non linear optimization problem with 3R planar joints"
    topics = ("robotics", "non-linear-optimization", "inverse-kinematics")
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "parallel": [False, "tbb", "openmp"],
        "neon": [False, True],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "parallel": False,
        "neon": True 
    }
    _cmake = None
    generators = "cmake_find_package"


    def config_options(self):
        return super().config_options()
        if self.settings.os == "Windows":
            del self.options.fPIC
        elif self.settings.os == "Linux":
            pass
    

    def source(self):
        tools.get(**self.conan_data["sources"][self.version])

    def requirements(self):
        self.requires("eigen/3.4.0")

    def build(self):
        cmake = CMake(self)
        cmake.build()
        cmake.install()

