# Quickstart: Physical AI and Humanoid Robotics Book

## Prerequisites
- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control

## Setup Instructions

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd frontend
   ```

2. **Install dependencies**
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Start the development server**
   ```bash
   npm start
   # or
   yarn start
   ```

4. **Access the book**
   - Open your browser to http://localhost:3000
   - The book homepage will display the title "Physical AI and Humanoid Robotics" with a centered "Read" button
   - Click the "Read" button to access the documentation

## Building for Production

```bash
npm run build
# or
yarn build
```

## Deployment

The site is configured for GitHub Pages deployment. After building, the static files will be in the `build` directory.

## Adding New Content

1. Create new markdown files in the `docs/` directory following the module/chapter structure
2. Update `sidebars.js` to include the new content in the navigation
3. Restart the development server to see changes

## Customizing the Theme

1. Modify `src/css/custom.css` to adjust colors and styling
2. Update `docusaurus.config.js` to change site metadata and theme settings
3. Customize the homepage in `src/pages/index.js`