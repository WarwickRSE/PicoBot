# Base ROS with GUI tools (for future RViz/etc.)
FROM ros:humble

# Set environment
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install Node.js (LTS) + serve for static frontend
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get update && apt-get install -y nodejs && \
    npm install -g serve && \
    rm -rf /var/lib/apt/lists/*

# Create workspace for GUI
WORKDIR /usr/src/app

# Copy package.json and install dependencies
COPY ./frontend/package*.json ./
RUN npm install

# Copy the rest of the frontend source
COPY ./frontend .

# Build the frontend (assumes you're using a framework like React/Vite)
RUN npm run build

# Serve the built site (on port 3000 by default)
EXPOSE 3000
CMD ["serve", "-s", "dist", "-l", "3000"]
