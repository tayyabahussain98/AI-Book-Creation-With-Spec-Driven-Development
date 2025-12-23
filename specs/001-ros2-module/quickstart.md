# Quickstart: Docusaurus Development Workflow

**Date**: 2025-12-21
**Feature**: 001-ros2-module
**Audience**: Content authors and developers

## Prerequisites

- Node.js 18+ installed
- Git configured
- Access to repository: `AI-book-creation-with-spec-driven-development`

---

## Initial Setup

### 1. Clone Repository (if not already done)

```bash
git clone https://github.com/tayyabahussain98/AI-book-creation-with-spec-driven-development.git
cd AI-book-creation-with-spec-driven-development
```

### 2. Switch to Feature Branch

```bash
git checkout 001-ros2-module
```

### 3. Initialize Docusaurus (First Time Only)

```bash
cd book_frontend
npx create-docusaurus@latest . classic --typescript=false

# Install dependencies
npm install
```

### 4. Configure for GitHub Pages

Edit `docusaurus.config.js`:

```javascript
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Building Intelligent Embodied Systems',
  url: 'https://tayyabahussain98.github.io',
  baseUrl: '/AI-book-creation-with-spec-driven-development/',
  organizationName: 'tayyabahussain98',
  projectName: 'AI-book-creation-with-spec-driven-development',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  // ... rest of config
};
```

---

## Development Workflow

### Start Local Development Server

```bash
cd book_frontend
npm start
```

- Opens browser at `http://localhost:3000`
- Live reload on file changes
- See errors/warnings in terminal

### Add New Chapter

1. Create markdown file in `docs/module-1-ros2/`:
   ```bash
   touch docs/module-1-ros2/chapter-{N}-{slug}.md
   ```

2. Add front matter:
   ```markdown
   ---
   sidebar_position: {N}
   ---

   # Chapter {N}: {Title}
   ```

3. Update `sidebars.js` if needed (auto-detected by default)

4. Write content following chapter template (see contracts/)

### Add Code Example

1. Create Python file in `docs/code-examples/module-1/`:
   ```bash
   touch docs/code-examples/module-1/{example_name}.py
   ```

2. Write code following pattern from research.md

3. Reference in chapter markdown:
   ````markdown
   ```python title="publisher_example.py"
   {CODE HERE}
   ```

   **Run this example**:
   ```bash
   python3 docs/code-examples/module-1/publisher_example.py
   ```
   ````

### Add Diagram Specification

1. Create markdown file in `docs/diagrams/module-1/`:
   ```bash
   touch docs/diagrams/module-1/{diagram-id}.md
   ```

2. Follow diagram spec structure from data-model.md

3. Reference in chapter:
   ```markdown
   See [Signal Flow Diagram](../diagrams/module-1/signal-flow.md)
   ```

---

## Testing & Validation

### Build Site Locally

```bash
cd book_frontend
npm run build
```

- Catches broken links
- Validates markdown syntax
- Outputs to `build/` directory

### Serve Production Build

```bash
npm run serve
```

- Test production build at `http://localhost:3000`
- Verify navigation, links, code highlighting

### Run Linter (Optional)

```bash
npm run lint
```

---

## Deployment to GitHub Pages

### Manual Deployment

```bash
cd book_frontend

# Ensure clean build
npm run build

# Deploy to gh-pages branch
USE_SSH=true npm run deploy

# Or with HTTPS:
GIT_USER=tayyabahussain98 npm run deploy
```

### Automated Deployment (CI/CD - Future)

Add GitHub Actions workflow (not implemented yet):

```yaml
# .github/workflows/deploy-docs.yml
name: Deploy Docusaurus

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - run: cd book_frontend && npm ci
      - run: cd book_frontend && npm run build
      - run: cd book_frontend && npm run deploy
```

---

## Common Issues & Solutions

### Issue: "Module not found" Error

**Solution**: Install dependencies
```bash
cd book_frontend
npm install
```

### Issue: Broken Links After Adding Chapter

**Cause**: Incorrect path in markdown link
**Solution**: Use relative paths:
- Same directory: `[link](./file.md)`
- Parent directory: `[link](../file.md)`
- Code examples: `[link](../code-examples/module-1/example.py)`

### Issue: Code Block Not Highlighting

**Cause**: Missing language identifier
**Solution**: Add language after triple backticks:
````
```python
# code here
```
````

### Issue: Sidebar Not Showing New Chapter

**Solution**: Check `sidebars.js` or add `sidebar_position` front matter

---

## File Structure Reference

```
book_frontend/
├── docs/
│   ├── module-1-ros2/
│   │   ├── chapter-1-intro-physical-ai.md
│   │   ├── chapter-2-ros2-architecture.md
│   │   ├── chapter-3-python-nodes.md
│   │   ├── chapter-4-packages-launch.md
│   │   └── chapter-5-urdf.md
│   ├── code-examples/
│   │   └── module-1/
│   │       ├── publisher_example.py
│   │       ├── subscriber_example.py
│   │       └── ...
│   └── diagrams/
│       └── module-1/
│           ├── signal-flow.md
│           └── ...
├── docusaurus.config.js  # Main configuration
├── sidebars.js           # Sidebar navigation
├── package.json          # Dependencies and scripts
├── static/               # Images, assets
├── src/                  # Custom React components
└── build/                # Production build output (gitignored)
```

---

## Useful Commands

| Command | Description |
|---------|-------------|
| `npm start` | Start dev server |
| `npm run build` | Build production site |
| `npm run serve` | Serve production build |
| `npm run clear` | Clear cache |
| `npm run deploy` | Deploy to GitHub Pages |

---

## Next Steps After Module 1 Complete

1. **Review**: Check all chapters build without errors
2. **Test**: Verify all code examples run on Humble/Iron
3. **Validate**: Ensure FR-001 through FR-014 satisfied
4. **Deploy**: Push to GitHub Pages via `npm run deploy`
5. **Iterate**: Gather feedback, refine content

---

## Resources

- **Docusaurus Docs**: https://docusaurus.io/docs
- **Markdown Guide**: https://docusaurus.io/docs/markdown-features
- **GitHub Pages**: https://docs.github.com/en/pages
- **ROS 2 Docs**: https://docs.ros.org/en/humble/
