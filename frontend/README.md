# Physical AI and Humanoid Robotics Book

This is a comprehensive technical book on Physical AI and Humanoid Robotics, built with Docusaurus.

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager

## Installation

1. Clone the repository
2. Navigate to the `frontend` directory
3. Install dependencies:

```bash
npm install
```

## Development

To start the development server:

```bash
npm start
```

This will start the Docusaurus development server and open the book in your browser at `http://localhost:3000`.

## Building for Production

To build the static site for production:

```bash
npm run build
```

The built site will be in the `build` directory.

## Project Structure

- `docs/` - Contains all the book content organized by modules and chapters
- `src/` - Contains custom React components, pages, and CSS
- `static/` - Contains static assets like images
- `docusaurus.config.js` - Main Docusaurus configuration file
- `sidebars.js` - Navigation sidebar configuration

## Contributing

To add new content:
1. Create new markdown files in the appropriate module directory in `docs/`
2. Update `sidebars.js` to include the new content in the navigation
3. Restart the development server to see changes

## Deployment

The site is configured for GitHub Pages deployment. After building, the static files can be deployed to any web server.