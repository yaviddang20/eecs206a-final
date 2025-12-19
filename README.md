# EECS 206A Final Project

This is a standalone repository for the EECS 206A Final Project website.

## Structure

- `index.md` - Main project page
- `assets/` - CSS, images, JavaScript, and 3D models
- `viewer/` - 3D viewer for Gaussian Splatting visualization
- `_layouts/` - Jekyll layout templates

## Adding Images

To add images to this project page:

1. Add your images to `assets/images/`
2. Update the image paths in `index.md` using:
   ```html
   <img src="{{ '/assets/images/your-image.png' | relative_url }}" alt="Description">
   ```

## Customization

Edit `index.md` to update:
- Project description
- Methodology steps
- Results and images
- Technology stack
- Links and resources
