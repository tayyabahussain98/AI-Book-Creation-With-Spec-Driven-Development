# Implementation Plan: Premium Reading UI for Physical AI Book

**Branch**: `006-premium-reading-ui` | **Date**: 2025-12-24 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/sp.specify` command

## Summary

Implement a premium, book-style UI for the Physical AI Book documentation site using Docusaurus Classic theme with custom CSS. The design features a soft teal (#26A69A) and warm yellow (#FFC857) color palette (no blue), Inter typography, robotics visual icons, and consistent branding with logo and favicon. The implementation focuses on CSS custom properties for theming, minimal DOM changes, and optimized SVG assets.

## Technical Context

**Language/Version**: CSS Custom Properties, TypeScript (Docusaurus config), React 18 (Docusaurus)
**Primary Dependencies**: Docusaurus 3.9.2, @docusaurus/theme-classic, Infima CSS framework
**Storage**: N/A (static site, assets in `img/` directory)
**Testing**: `npm run build`, browser visual inspection, dev tools audit
**Target Platform**: Modern web browsers with CSS custom properties support
**Project Type**: Single web application (Docusaurus documentation site)
**Performance Goals**: Homepage load <3s, CSS rendering <100ms, SVG icons <10KB each
**Constraints**: Classic Docusaurus theme only, styling via `src/css/custom.css`, no animation-heavy UI, optimized images only
**Scale/Scope**: Single documentation site with 4 main modules, ~50+ chapter pages

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Spec-Driven Development | ✅ PASS | Complete spec exists with 6 user stories, 20 FRs, 8 SCs |
| II. Source-Grounded Accuracy | N/A | UI feature - no content/RAG components |
| III. Verifiable and Runnable Code | ✅ PASS | `npm run build` validates CSS and TypeScript |
| IV. Clear Separation of Concerns | ✅ PASS | Specs in `specs/`, source in `book_frontend/`, PHRs in `history/` |
| V. AI-Native Architecture | N/A | UI feature - no AI/RAG components |
| VI. Free-Tier and Security Constraints | ✅ PASS | No external services, CSS-only changes |
| VII. Smallest Viable Changes | ✅ PASS | Single CSS file + config updates + homepage component |

## Project Structure

### Documentation (this feature)

```text
specs/006-premium-reading-ui/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (below)
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
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
```

**Structure Decision**: Docusaurus Classic theme structure with customizations in `src/css/custom.css` for theming, `docusaurus.config.ts` for branding, and `src/pages/index.tsx` for homepage redesign. All changes are isolated to the `book_frontend` directory.

## Research Findings (Phase 0)

### Docusaurus Navbar Logo Configuration

**Decision**: Use `docusaurus.config.ts` navbar.logo configuration with custom SVG logo

**Rationale**: Docusaurus supports SVG logos natively, allows alt text, and handles dark mode variants automatically via class-based src attributes.

**Implementation**:
```typescript
// docusaurus.config.ts
navbar: {
  logo: {
    alt: 'Physical AI Book Logo',
    src: 'img/logo.svg',
    // Dark mode variant not needed - single SVG with CSS-filtered colors
  },
}
```

### Docusaurus Favicon Configuration

**Decision**: Configure favicon in `docusaurus.config.ts` with path to `static/img/favicon.ico`

**Rationale**: Docusaurus reads favicon from static directory. An ICO file ensures broad browser compatibility.

**Implementation**:
```typescript
// docusaurus.config.ts
favicon: 'img/favicon.ico',
```

### CSS Custom Properties in Docusaurus

**Decision**: Override Infima CSS variables in `:root` and `[data-theme='dark']` in `src/css/custom.css`

**Rationale**: Docusaurus uses Infima for styling. CSS custom properties are the recommended override mechanism. No JavaScript required.

**Implementation**: Define all design tokens in custom.css:
- Color palette: #26A69A (primary), #FFC857 (accent)
- Backgrounds: #F8FAFC (light), #0B1220 (dark)
- Typography: Inter/system-ui, spacing scale, shadows

### SVG Icon Optimization

**Decision**: Use inline SVG icons in React components, optimized SVGO style

**Rationale**: Inline SVGs allow CSS styling via fill/currentColor, eliminate network requests, and enable tree-shaking.

**Optimization approach**:
- Remove XML declarations, comments, unused namespaces
- Simplify paths, use relative coordinates
- Set `fill="currentColor"` for CSS color control
- Keep under 2KB per icon (4 icons = <8KB total)

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None | N/A | N/A |

All implementation follows smallest-change principle within Docusaurus Classic theme constraints.

## Phase 1: Design Artifacts

### data-model.md (Generated)

See `data-model.md` for CSS custom properties design tokens.

### quickstart.md (Generated)

See `quickstart.md` for development commands.

## Next Steps

After `/sp.plan` completes:
1. Run `/sp.tasks` to generate implementation task list
2. Run `/sp.implement` to execute tasks sequentially
3. Validate with `npm run build` after each major change
