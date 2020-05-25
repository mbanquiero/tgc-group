
//Matrices de transformacion
float4x4 matWorld; //Matriz de transformacion World
float4x4 matWorldView; //Matriz World * View
float4x4 matWorldViewProj; //Matriz World * View * Projection
float4x4 matInverseTransposeWorld; //Matriz Transpose(Invert(World))
//Factor de translucidez
float alphaValue = 1;
float3 lightPosition = float3(1000,1000,1000);
float time = 0;

//Textura para DiffuseMap
texture texDiffuseMap;
sampler2D diffuseMap = sampler_state
{
    Texture = (texDiffuseMap);
    ADDRESSU = WRAP;
    ADDRESSV = WRAP;
    MINFILTER = LINEAR;
    MAGFILTER = LINEAR;
    MIPFILTER = LINEAR;
};
texture texDiffuseMap2;
sampler2D diffuseMap2 = sampler_state
{
    Texture = (texDiffuseMap2);
    ADDRESSU = MIRROR;
    ADDRESSV = MIRROR;
    MINFILTER = LINEAR;
    MAGFILTER = LINEAR;
    MIPFILTER = LINEAR;
};

texture texDiffuseMap3;
sampler2D diffuseMap3 = sampler_state
{
    Texture = (texDiffuseMap3);
    ADDRESSU = MIRROR;
    ADDRESSV = MIRROR;
    MINFILTER = LINEAR;
    MAGFILTER = LINEAR;
    MIPFILTER = LINEAR;
};

texture texColorMap;
sampler2D colorMap = sampler_state
{
    Texture = (texColorMap);
    ADDRESSU = WRAP;
    ADDRESSV = WRAP;
    MINFILTER = LINEAR;
    MAGFILTER = LINEAR;
    MIPFILTER = LINEAR;
};


struct VS_INPUT
{
    float4 Position : POSITION0;
    float2 Texcoord : TEXCOORD0;
    float3 Normal : NORMAL0;
};

struct VS_OUTPUT
{
    float4 Position : POSITION0;
    float2 Texcoord : TEXCOORD0;
    float3 WorldPos : TEXCOORD1;
    float3 WorldNormal : TEXCOORD2;
	

};



VS_OUTPUT vs_RenderTerrain(VS_INPUT input)
{
    VS_OUTPUT output;

	//Proyectar posicion
    output.Position = mul(input.Position, matWorldViewProj);

	//Enviar Texcoord directamente
    output.Texcoord = input.Texcoord;

    output.WorldPos = mul(input.Position, matWorld);
    output.WorldNormal = mul(input.Normal, matInverseTransposeWorld).xyz;


    return output;
}

struct PS_INPUT
{
    float2 Texcoord : TEXCOORD0;
    float3 WorldPos : TEXCOORD1;
    float3 WorldNormal : TEXCOORD2;

};

float hmin = 6*1/256;
float hmax = 6*254/256;



//Pixel Shader
float4 ps_RenderTerrain(PS_INPUT input) : COLOR0
{
	//float h = lerp(0.2, 0.6 , (input.WorldPos.y-hmin)/(hmax-hmin));
	//float3 lightPosition = float3(1000*abs(cos(time)),1000*abs(sin(time)),0);

    float3 N = normalize(input.WorldNormal);
    float3 L = normalize(lightPosition - input.WorldPos);
	float kd = saturate( 0.4 + 0.7*saturate(dot(N, L)));
	
	float3 c = tex2D(colorMap, input.Texcoord).rgb;
	float3 tex1 = tex2D(diffuseMap, input.Texcoord*31).rgb;
    float3 tex2 = tex2D(diffuseMap2, input.Texcoord*27).rgb;
    float3 tex3 = tex2D(diffuseMap3, input.Texcoord*17).rgb;
	float3 clr = lerp ( lerp(tex1 , tex3, c.r), c , 0.3);
	return float4(clr*kd, 1);
	
}

technique RenderTerrain
{
    pass Pass_0
    {
        VertexShader = compile vs_3_0 vs_RenderTerrain();
        PixelShader = compile ps_3_0 ps_RenderTerrain();
    }
}

