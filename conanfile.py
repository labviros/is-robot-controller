from conans import ConanFile, CMake, tools


class IsRobotControllerServiceConan(ConanFile):
    name = "is-robot-controller"
    version = "0.0.1"
    license = "MIT"
    url = "https://github.com/labviros/is-robot-controller"
    description = ""
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "build_tests": [True, False],
    }
    default_options = "shared=False", "fPIC=True", "build_tests=False"
    generators = "cmake", "cmake_find_package", "cmake_paths"
    requires = (
        "is-msgs/1.1.8@is/stable",
        "is-wire/1.1.4@is/stable",
        "eigen/3.3.5@conan/stable",
    )

    exports_sources = "*"

    def build_requirements(self):
        if self.options.build_tests:
            self.build_requires("gtest/1.8.0@bincrafters/stable")

    def configure(self):
        self.options["is-msgs"].shared = True

    def build(self):
        cmake = CMake(self, generator='Ninja')
        cmake.definitions[
            "CMAKE_POSITION_INDEPENDENT_CODE"] = self.options.fPIC
        cmake.definitions["enable_tests"] = self.options.build_tests
        cmake.configure()
        cmake.build()
        # if self.options.build_tests:
        #     cmake.test()

    def package_info(self):
        pass
