// std
#include <cstdlib>

// yaml_tools
#include "yaml_tools/yaml_tools.hpp"

int main(int argc, char** argv) {
  if (argc != 3) {
    MELO_ERROR("Start with: ./format_file <input_file> <output_file>")
    return EXIT_FAILURE;
  }
  const std::string inputFile = argv[1];   // NOLINT
  const std::string outputFile = argv[2];  // NOLINT
  try {
    yaml_tools::YamlNode::fromFile(inputFile).toFile(outputFile);
  } catch (const yaml_tools::Exception& exception) {
    MELO_ERROR("Caught an exception while formatting file: %s.", exception.what())
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
