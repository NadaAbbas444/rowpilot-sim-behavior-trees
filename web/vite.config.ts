/// <reference types="vitest" />
import react from '@vitejs/plugin-react';
import { defineConfig } from 'vite';

// `npm run dev` serves the page on :5173 and passes /api (including the run WebSocket) to the
// Python server on :8000 (`python -m rowpilot editor`).
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: { '/api': { target: 'http://127.0.0.1:8000', ws: true } },
  },
  test: { environment: 'jsdom', globals: true },
});
