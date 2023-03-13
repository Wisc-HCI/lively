module.exports = function(context, options) {
    return {
      name: 'loaders',
      configureWebpack(config, isServer) {
        return {
          experiments: {asyncWebAssembly: true, syncWebAssembly: true},
          module: {
            rules: [
              {
                test: /\.(glb|gltf)$/i,
                use: ['file-loader'],
              },
            ],
          },
        };
      },
    };
  };