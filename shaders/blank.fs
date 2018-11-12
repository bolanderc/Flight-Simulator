#version 330
in vec2 newTexture;

out vec4 outColor;
uniform sampler2D samplerTexture;

void main()
{


	vec4 texel = texture(samplerTexture, newTexture);
	if (texel.a <0.5)
		discard;

	outColor = texel;
}
