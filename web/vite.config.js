import { defineConfig } from 'vite'

export default defineConfig({
  build: {
    outDir: '../resources/system/www',
    emptyOutDir: true,
    rollupOptions: {
      output: {
        // 移除文件名中的 Hash，缩短长度
        entryFileNames: `[name].js`,
        chunkFileNames: `[name].js`,
        assetFileNames: `[name].[ext]`
      }
    }
  }
})