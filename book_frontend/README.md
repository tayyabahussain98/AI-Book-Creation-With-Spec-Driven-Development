# Physical AI & Humanoid Robotics Book

This is the educational book website for "Physical AI & Humanoid Robotics", built using [Docusaurus](https://docusaurus.io/).

## Quick Start

### Prerequisites
- Node.js 18.0 or higher
- npm 9.0 or higher

### Installation

```bash
npm install
```

### Local Development

```bash
npm start
```

This command starts a local development server and opens `http://localhost:3000` in your browser. Most changes are reflected live without having to restart the server.

### Build

```bash
npm run build
```

This command generates static content into the `build` directory. Build time: ~30 seconds.

### Test Production Build Locally

```bash
npm run serve
```

This command serves the production build locally at `http://localhost:3000`.

## Project Structure

```
book_frontend/
├── docs/                          # Book content (Markdown)
│   ├── module-1-ros2/            # Module 1 chapters
│   │   ├── chapter-1-intro-physical-ai.md
│   │   ├── chapter-2-ros2-architecture.md
│   │   ├── chapter-3-python-nodes.md
│   │   ├── chapter-4-packages-launch.md
│   │   └── chapter-5-urdf.md
│   ├── code-examples/            # Runnable code examples
│   │   └── module-1/
│   │       ├── publisher_example.py
│   │       ├── subscriber_example.py
│   │       ├── param_logger_example.py
│   │       ├── simple_launch.py
│   │       ├── ai_agent_node.py
│   │       └── simple_arm.urdf
│   └── diagrams/                 # Diagram specifications
│       └── module-1/
│           ├── physical-ai-components.md
│           ├── node-lifecycle.md
│           ├── signal-flow.md
│           ├── pubsub-interaction.md
│           ├── package-structure.md
│           ├── ai-ros-integration.md
│           ├── urdf-tree.md
│           └── joint-types.md
├── src/                          # React components
├── static/                       # Static assets
├── docusaurus.config.ts          # Docusaurus configuration
└── sidebars.ts                   # Sidebar navigation
```

## Module 1: The Robotic Nervous System (ROS 2)

Module 1 covers the fundamentals of ROS 2 for robotics and AI integration:

1. **Chapter 1**: Introduction to Physical AI - Middleware rationale, ROS 2 overview
2. **Chapter 2**: ROS 2 Architecture - Nodes, topics, services, actions, DDS
3. **Chapter 3**: Python Nodes - Publisher/subscriber patterns, parameters, logging
4. **Chapter 4**: Packages and Launch Files - AI agent integration, multi-node systems
5. **Chapter 5**: URDF and Robot Description - Kinematic modeling for simulation

## Deployment

### GitHub Pages

1. Configure `docusaurus.config.ts` with your GitHub repository details
2. Deploy with:

```bash
npm run deploy
```

This builds the site and pushes to the `gh-pages` branch.

### Manual Deployment

```bash
npm run build
# Then upload the build/ directory to your hosting service
```

## Development

### Adding a New Chapter

1. Create markdown file in `docs/module-X/`
2. Add to `sidebars.ts` navigation
3. Include front matter with `sidebar_position` and `title`

### Adding Code Examples

1. Place code in `docs/code-examples/module-X/`
2. Embed in chapter using markdown code blocks with language tags
3. Validate syntax with appropriate linter

### Adding Diagrams

1. Create diagram specification in `docs/diagrams/module-X/`
2. Use ASCII art, mermaid, or markdown tables
3. Reference from chapter with relative links

## Troubleshooting

### Build Fails

- Clear cache: `rm -rf .docusaurus/ build/`
- Reinstall dependencies: `rm -rf node_modules && npm install`
- Check for syntax errors in markdown files

### Port Already in Use

- Change port: `npm start -- --port 3001`

## License

See LICENSE file in repository root.
