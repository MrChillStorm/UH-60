$FG_GLSL_VERSION

// Shaders/HDR/model_default.frag with emission: the texture times lit_color (sRGB), lit_level and lit_gain (cd/m2), for
// Models/Effects/Lit/lit.eff

in VS_OUT {
    float flogz;
    vec2 texcoord;
    vec3 vertex_normal;
    vec4 material_color;
} fs_in;

uniform sampler2D color_tex;

uniform float alpha_cutoff;
uniform float pbr_metallic;
uniform float pbr_roughness;
uniform vec3 lit_color;
uniform float lit_level;
uniform float lit_gain;

// gbuffer_pack.glsl
void gbuffer_pack_pbr_opaque(vec3 normal,
                             vec3 base_color,
                             float metallic,
                             float roughness,
                             float occlusion,
                             vec3 emissive);
// color.glsl
vec3 eotf_inverse_sRGB(vec3 srgb);
// logarithmic_depth.glsl
float logdepth_encode(float z);

void main()
{
    vec4 texel = texture(color_tex, fs_in.texcoord);
    if (texel.a * fs_in.material_color.a < alpha_cutoff)
        discard;
    vec3 texel_linear = eotf_inverse_sRGB(texel.rgb);
    vec3 color = texel_linear * fs_in.material_color.rgb;
    vec3 emissive = texel_linear * eotf_inverse_sRGB(lit_color) * max(lit_level * lit_gain, 0.0);
    vec3 N = normalize(fs_in.vertex_normal);
    gbuffer_pack_pbr_opaque(N, color, pbr_metallic, pbr_roughness, 1.0, emissive);
    gl_FragDepth = logdepth_encode(fs_in.flogz);
}
