import { defineConfig } from 'vite';
import wasm from 'vite-plugin-wasm';
import topLevelAwait from 'vite-plugin-top-level-await';
import path from 'path';

// https://vitejs.dev/config/
export default defineConfig({
  base: '/Basic_Initialization/',
  plugins: [wasm(), topLevelAwait()],
  worker: {
    plugins: [wasm(), topLevelAwait()],
  },
});
