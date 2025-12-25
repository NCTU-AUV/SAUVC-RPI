# Use Node.js 18 on Alpine Linux
FROM node:18-alpine

# Set working directory
WORKDIR /app

# Copy package.json and package-lock.json (or bun.lockb if used)
COPY package.json ./

# Install dependencies using npm (since we are creating a generic node setup, assuming npm is fine even if bun is present, usually safest for broad compat unless user insists on bun)
# However, I see bun.lockb in the file list. It's better to use bun if available, or just stick to npm if we don't want to install bun. Use npm for simplicity as per plan.
# If bun.lockb is present but no package-lock.json, npm install might regenerate lockfile.
RUN npm install

# Copy source code
COPY . .

# Expose port 8080
EXPOSE 8080

# Start the application
CMD ["npm", "run", "dev", "--", "--host"]
