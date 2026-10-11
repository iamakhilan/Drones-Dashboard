/** @type {import('tailwindcss').Config} */
module.exports = {
  content: ["./index.html", "./js/**/*.js"],
  theme: {
    extend: {
      colors: {
        brand: { 50: '#F0F9FF', 100: '#E0F2FE', 500: '#0EA5E9', 600: '#0284C7', 700: '#0369A1' }
      },
      fontFamily: {
        sans: ['Inter','-apple-system','BlinkMacSystemFont','Segoe UI','Roboto','Helvetica','Arial','sans-serif'],
        mono: ['SF Mono','Monaco','Menlo','Consolas','Ubuntu Mono','monospace']
      }
    }
  },
  plugins: [require('@tailwindcss/forms')],
  corePlugins: { preflight: true }
}
