# Quickstart: Premium Reading UI Development

## Prerequisites

- Node.js 18+
- npm 9+
- Git

## Installation

```bash
cd book_frontend
npm install
```

## Development Commands

### Start Development Server

```bash
npm run start
```

Runs the Docusaurus development server at `http://localhost:3000`

### Build for Production

```bash
npm run build
```

Generates static files in `build/` directory. Validates CSS and TypeScript.

### Clear Cache

```bash
npm run clear
```

Clears Docusaurus cache when experiencing build issues.

### Serve Production Build Locally

```bash
npm run serve
```

Serves the production build at `http://localhost:3000` for testing.

## File Structure

```
book_frontend/
├── src/
│   ├── css/
│   │   └── custom.css           # Main theme file (CSS custom properties)
│   └── pages/
│       ├── index.tsx            # Homepage (hero + module cards)
│       └── index.module.css     # Homepage component styles
├── docusaurus.config.ts         # Theme config, logo, favicon
├── static/
│   └── img/
│       ├── logo.svg             # Navbar logo (teal/yellow palette)
│       └── favicon.ico          # Browser favicon
└── build/                       # Production build output
```

## CSS Template Structure

All theme customizations should follow this structure:

```css
/* 1. CSS Custom Properties - Design Tokens */
/*    Define all color, typography, spacing tokens */

/* 2. Docusaurus Infima Overrides */
/*    Override framework variables */

/* 3. Global Typography */
/*    Base styles for body, headings, links */

/* 4. Homepage Styles */
/*    Hero, module cards, sections */

/* 5. Sidebar Styles */
/*    Navigation, active states */

/* 6. Content Styling */
/*    Tables, blockquotes, admonitions */

/* 7. Responsive Breakpoints */
/*    Mobile, tablet, desktop */

/* 8. Navbar & Footer */
/*    Brand, navigation, footer */
```

## Color Palette Reference

| Role | Light Mode | Dark Mode |
|------|------------|-----------|
| Primary | `#26A69A` | `#26A69A` |
| Accent | `#FFC857` | `#FFC857` |
| Background | `#F8FAFC` | `#0B1220` |
| Surface | `#FFFFFF` | `#1A2332` |
| Text Primary | `#1F2937` | `#F3F4F6` |

## Testing Checklist

- [ ] `npm run build` completes without errors
- [ ] Homepage displays hero and module cards
- [ ] Light/dark mode toggle works
- [ ] Module cards link to correct chapter pages
- [ ] Sidebar shows active state for current page
- [ ] Mobile view stacks cards vertically
- [ ] Logo displays in navbar
- [ ] Favicon appears in browser tab
- [ ] No blue colors in design (use browser devtools to verify)
- [ ] SVG icons render correctly
- [ ] Page load time under 3 seconds

## Common Issues

### Changes Not Appearing

```bash
npm run clear
npm run start
```

### Build Failures

Check `docusaurus.config.ts` for TypeScript errors:
```bash
npx tsc --noEmit
```

### CSS Not Loading

Ensure `custom.css` is referenced in `docusaurus.config.ts`:
```typescript
theme: {
  customCss: './src/css/custom.css',
},
```

## Deployment

Push to GitHub to trigger GitHub Pages deployment:

```bash
git add .
git commit -m "feat: implement premium reading UI"
git push origin 006-premium-reading-ui
```

Create a pull request to merge into the main branch.
