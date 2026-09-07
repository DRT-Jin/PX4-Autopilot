variable "SIMULATOR" {
  default = "sih"
}

group "default" {
  targets = ["sitl", "ros2"]
}

target "sitl" {
  context    = "docker-context"
  dockerfile = "Dockerfile.${SIMULATOR}"
}

target "ros2" {
  context    = "docker-context"
  dockerfile = "Dockerfile.ros2"
  contexts = {
    sitl = "target:sitl"
  }
}
