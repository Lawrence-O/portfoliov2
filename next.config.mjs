/** @type {import('next').NextConfig} */
const nextConfig = {
  images: {
    // Enable modern image formats for smaller file sizes
    formats: ['image/avif', 'image/webp'],
  },
};

export default nextConfig;