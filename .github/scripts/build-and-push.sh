#!/bin/bash

# Build and push Docker image to GitHub Container Registry (GHCR)
# Usage: ./build-and-push.sh <tag>
# Example: ./build-and-push.sh v1.0.0

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if tag argument is provided
if [ $# -eq 0 ]; then
    print_error "No tag provided!"
    echo "Usage: $0 <tag>"
    echo "Example: $0 v1.0.0"
    exit 1
fi

TAG=$1

# Validate tag format (should not be empty and should not contain spaces)
if [[ -z "$TAG" || "$TAG" =~ [[:space:]] ]]; then
    print_error "Invalid tag format: '$TAG'"
    echo "Tag should not be empty and should not contain spaces"
    exit 1
fi

# Get repository info
REPO_OWNER=$(git config --get remote.origin.url | sed 's/.*github\.com[:/]\([^/]*\)\/.*/\1/')
REPO_NAME=$(git config --get remote.origin.url | sed 's/.*\/\([^/]*\)\.git.*/\1/')

if [[ -z "$REPO_OWNER" || -z "$REPO_NAME" ]]; then
    print_error "Could not determine repository owner and name from git remote"
    print_info "Make sure you're in a git repository with a GitHub remote"
    exit 1
fi

# Define image name
IMAGE_NAME="ghcr.io/${REPO_OWNER}/${REPO_NAME}"
FULL_IMAGE_TAG="${IMAGE_NAME}:${TAG}"
LATEST_TAG="${IMAGE_NAME}:latest"

print_info "Repository: ${REPO_OWNER}/${REPO_NAME}"
print_info "Building image: ${FULL_IMAGE_TAG}"

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    print_error "Docker is not running or not accessible"
    print_info "Please start Docker and ensure you have proper permissions"
    exit 1
fi

# Check if logged into GHCR
print_info "Checking GHCR authentication..."
if ! echo "$GITHUB_TOKEN" | docker login ghcr.io -u "$GITHUB_ACTOR" --password-stdin > /dev/null 2>&1; then
    print_warning "Not authenticated with GHCR or missing environment variables"
    print_info "Please ensure GITHUB_TOKEN and GITHUB_ACTOR are set, or run:"
    print_info "echo \$GITHUB_TOKEN | docker login ghcr.io -u \$GITHUB_ACTOR --password-stdin"
    print_info "Or use: docker login ghcr.io"
    
    # Try interactive login if environment variables are not set
    if [[ -z "$GITHUB_TOKEN" || -z "$GITHUB_ACTOR" ]]; then
        print_info "Attempting interactive login to GHCR..."
        if ! docker login ghcr.io; then
            print_error "Failed to login to GHCR"
            exit 1
        fi
    else
        exit 1
    fi
fi

# Change to the repository root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "$REPO_ROOT"

print_info "Working directory: $(pwd)"

# Check if Dockerfile exists
DOCKERFILE_PATH="dirac_deployment/Dockerfile"
if [[ ! -f "$DOCKERFILE_PATH" ]]; then
    print_error "Dockerfile not found at: $DOCKERFILE_PATH"
    exit 1
fi

print_info "Using Dockerfile: $DOCKERFILE_PATH"

# Build the Docker image
print_info "Building Docker image..."
if docker build -f "$DOCKERFILE_PATH" -t "$FULL_IMAGE_TAG" -t "$LATEST_TAG" .; then
    print_success "Docker image built successfully"
else
    print_error "Failed to build Docker image"
    exit 1
fi

# Push the tagged image
print_info "Pushing tagged image: $FULL_IMAGE_TAG"
if docker push "$FULL_IMAGE_TAG"; then
    print_success "Tagged image pushed successfully"
else
    print_error "Failed to push tagged image"
    exit 1
fi

# Push the latest tag
print_info "Pushing latest image: $LATEST_TAG"
if docker push "$LATEST_TAG"; then
    print_success "Latest image pushed successfully"
else
    print_error "Failed to push latest image"
    exit 1
fi

# Show image information
print_success "Build and push completed!"
print_info "Image details:"
echo "  - Tagged image: $FULL_IMAGE_TAG"
echo "  - Latest image: $LATEST_TAG"
echo "  - Registry: GitHub Container Registry (GHCR)"

# Show pull command
print_info "To pull this image:"
echo "  docker pull $FULL_IMAGE_TAG"
echo "  docker pull $LATEST_TAG"

# Clean up local images (optional)
read -p "Do you want to remove local images to save space? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    print_info "Removing local images..."
    docker rmi "$FULL_IMAGE_TAG" "$LATEST_TAG" || print_warning "Could not remove some images"
    print_success "Local images cleaned up"
fi

print_success "Script completed successfully!"
