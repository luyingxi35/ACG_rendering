#include "Texture.h"

Texture::Texture(const std::string& filepath) {
	int channels;
	data = stbi_load(filepath.c_str(), &width, &height, &channels, STBI_rgb);
	if (!data) {
		std::cerr << "Error loading texture: " << filepath << std::endl;
	}
}