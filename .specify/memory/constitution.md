<!--
Sync Impact Report:
Version: 1.0.0 → 1.0.0 (Initial Creation)
Modified Principles: New constitution created from user specifications
Added Sections: All core principles (I-VII), Technology Stack, Development Workflow, Governance
Removed Sections: None (initial creation)
Templates Status:
  ✅ plan-template.md - Constitution Check section references this file
  ✅ spec-template.md - User scenarios align with spec-driven principles
  ✅ tasks-template.md - Task structure supports testable, small changes
Follow-up TODOs: None
-->

# Physical AI Technical Book with RAG Chatbot Constitution

## Core Principles

### I. Spec-Driven Development (NON-NEGOTIABLE)
Specifications MUST be written before implementation begins. Every feature requires:
- Complete spec defining inputs, outputs, and constraints
- Implementation that strictly follows spec without deviation
- Clear separation between business requirements (spec) and technical implementation (plan)

**Rationale**: Ensures reproducibility, prevents scope creep, enables accurate validation against defined requirements.

### II. Source-Grounded Accuracy (NON-NEGOTIABLE)
All content and AI responses MUST be grounded in verifiable sources:
- RAG chatbot answers MUST cite only book content in vector database
- Selected-text Q&A MUST use only user-highlighted context
- No hallucinated APIs, behaviors, or information permitted
- Reject out-of-scope questions with safe fallback responses

**Rationale**: Maintains technical accuracy, builds reader trust, prevents misinformation in educational content.

### III. Verifiable and Runnable Code
All code examples and implementations MUST be:
- Executable without modification
- Tested and working before publication
- Documented with clear setup instructions
- Free of invented or undocumented APIs

**Rationale**: Ensures readers can reproduce results, learn effectively, and trust the content.

### IV. Clear Separation of Concerns
Project structure MUST maintain distinct boundaries:
- **Specs**: Business requirements, user scenarios, success criteria (tech-agnostic)
- **Documentation**: Book chapters, explanations, examples
- **Implementation**: Backend services, RAG pipeline, frontend components
- No mixing of concerns across these boundaries

**Rationale**: Enables independent evolution, simplifies maintenance, supports AI-native development workflow.

### V. AI-Native Architecture
System design MUST embrace AI-first patterns:
- Agent-based workflows where appropriate
- RAG for knowledge retrieval with proper grounding
- Tool use for extensibility
- Prompt engineering as first-class design concern

**Rationale**: Aligns with book's teaching goals, demonstrates modern AI practices, ensures technical credibility.

### VI. Free-Tier and Security Constraints
All infrastructure MUST respect operational boundaries:
- **Free-tier only**: Qdrant Cloud Free, Neon Serverless Postgres Free, GitHub Pages
- **No hardcoded secrets**: Environment variables for all credentials
- **Documented env vars**: Clear `.env.example` with all required configuration
- **Minimal dependencies**: Avoid paid or enterprise-only tools

**Rationale**: Ensures accessibility for readers following along, demonstrates cost-effective architecture, enforces security best practices.

### VII. Smallest Viable Changes
Development MUST follow incremental principles:
- Testable, atomic changes only
- Precise code references (file:line format) in all proposals
- No refactoring of unrelated code
- Keep reasoning internal; output only decisions and justifications

**Rationale**: Reduces risk, simplifies review, enables rollback, maintains focus on requirements.

## Technology Stack

**Book Platform**:
- Framework: Docusaurus
- Deployment: GitHub Pages
- Structure: Clean chapter navigation with embedded chatbot

**RAG Chatbot Backend**:
- API Framework: FastAPI
- Vector Database: Qdrant Cloud (Free tier)
- Relational Database: Neon Serverless Postgres (Free tier)
- AI SDKs: OpenAI Agents / ChatKit
- Embedding Model: OpenAI text-embedding-3-small (or similar free-tier option)
- LLM: OpenAI GPT-4 (with free-tier quotas)

**Requirements**:
- All services MUST operate within free-tier limits
- Environment variables for configuration
- Health checks and graceful degradation
- Local development support (Docker Compose recommended)

## Development Workflow

### Specification Phase
1. Create feature spec in `specs/<feature>/spec.md`
2. Define user scenarios with Given-When-Then format
3. List functional requirements (FR-XXX format)
4. Define measurable success criteria (SC-XXX format)
5. Get user approval before planning

### Planning Phase
1. Create implementation plan in `specs/<feature>/plan.md`
2. Complete research.md (technology evaluation, API exploration)
3. Define data-model.md (entities, relationships, constraints)
4. Document API contracts in `contracts/`
5. Run Constitution Check (must pass before proceeding)
6. Document architectural decisions in ADRs for significant choices

### Task Generation Phase
1. Generate task list in `specs/<feature>/tasks.md`
2. Organize by user story with independent test criteria
3. Mark parallel-safe tasks with [P] prefix
4. Include exact file paths and acceptance criteria
5. Each task must be atomic and independently testable

### Implementation Phase
1. Create Prompt History Record (PHR) for each development session
2. Follow Red-Green-Refactor cycle if tests required
3. Commit after each logical task completion
4. Reference constitution principles in commit messages
5. No deviation from spec without explicit amendment

### Quality Gates
- **Pre-implementation**: Spec approved, Constitution Check passed
- **Pre-commit**: Code follows smallest-change principle, no unrelated edits
- **Pre-deployment**: All success criteria met, RAG grounding verified
- **Post-deployment**: Book builds correctly, chatbot rejects out-of-scope queries

## Governance

This Constitution supersedes all other development practices and preferences. All changes, pull requests, and reviews MUST verify compliance with these principles.

**Amendment Process**:
1. Proposed changes require documentation of rationale and impact
2. Constitution version increments per semantic versioning:
   - MAJOR: Backward-incompatible principle changes
   - MINOR: New principles or material expansions
   - PATCH: Clarifications, wording fixes
3. Amendment requires explicit approval and sync with dependent templates
4. Migration plan required for any breaking changes

**Compliance**:
- All code reviews must reference relevant principles
- Complexity violations require explicit justification in plan.md
- PHRs must be created for every development session
- ADRs required for architecturally significant decisions
- Use CLAUDE.md for runtime development guidance

**Version**: 1.0.0 | **Ratified**: 2025-12-21 | **Last Amended**: 2025-12-21
