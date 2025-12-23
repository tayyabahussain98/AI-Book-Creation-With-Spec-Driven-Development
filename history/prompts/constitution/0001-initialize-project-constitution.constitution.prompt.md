---
id: 0001
title: Initialize Project Constitution
stage: constitution
date: 2025-12-21
surface: agent
model: claude-sonnet-4-5
feature: none
branch: master
user: win
command: /sp.constitution
labels: ["constitution", "governance", "principles", "setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A (constitution document)
---

## Prompt

Project: AI/Spec-Driven Technical Book with Embedded RAG Chatbot

Core principles:
- Spec-driven, reproducible development
- Source-grounded accuracy (no hallucinations)
- Clarity for developers and AI engineers
- AI-native architecture (agents, RAG, tools)

Key standards:
- All content and code must be verifiable and runnable
- No invented APIs or undocumented behavior
- Clear separation of specs, docs, and implementation
- RAG answers must be grounded only in book content
- Selected-text Q&A must use only user-selected context

Book requirements:
- Framework: Docusaurus
- Deployment: GitHub Pages
- Clean repo structure and navigation
- Chapters must include explanation + examples

RAG Chatbot requirements:
- Backend: FastAPI
- Vector DB: Qdrant Cloud (Free tier)
- Database: Neon Serverless Postgres
- SDKs: OpenAI Agents / ChatKit
- Reject out-of-context questions
- Provide safe fallback when no answer exists

Spec-Kit Plus rules:
- Specs written before implementation
- Each feature defines inputs, outputs, constraints
- Implementation must strictly follow specs

Constraints:
- Free-tier usage only
- No hardcoded secrets
- All env vars documented

Success criteria:
- Book builds and deploys correctly
- RAG chatbot works locally and in production
- Answers are accurate, grounded, and reproducible

## Response snapshot

Constitution created at `.specify/memory/constitution.md` with version 1.0.0.

Established seven core principles:
1. Spec-Driven Development (NON-NEGOTIABLE)
2. Source-Grounded Accuracy (NON-NEGOTIABLE)
3. Verifiable and Runnable Code
4. Clear Separation of Concerns
5. AI-Native Architecture
6. Free-Tier and Security Constraints
7. Smallest Viable Changes

Added sections:
- Technology Stack: Docusaurus, FastAPI, Qdrant Cloud, Neon Postgres, OpenAI SDKs
- Development Workflow: Specification → Planning → Task Generation → Implementation phases
- Governance: Amendment process, compliance requirements, versioning policy

All template dependencies verified:
✅ plan-template.md - Constitution Check section references this file
✅ spec-template.md - User scenarios align with spec-driven principles
✅ tasks-template.md - Task structure supports testable, small changes

## Outcome

- ✅ Impact: Project constitution established with 7 core principles and governance framework
- 🧪 Tests: N/A (governance document)
- 📁 Files: 1 created (.specify/memory/constitution.md)
- 🔁 Next prompts: Ready for feature specification via /sp.specify or planning via /sp.plan
- 🧠 Reflection: Constitution successfully captures project requirements with clear boundaries between book content, RAG chatbot, and spec-driven workflow. Free-tier constraint principle ensures accessibility for readers.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All placeholders filled, principles testable, templates synchronized
- Prompt variant (if applicable): N/A (initial constitution)
- Next experiment (smallest change to try): Test constitution gates during first feature specification to validate principle enforcement
