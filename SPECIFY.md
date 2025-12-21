# Project Specification Document
# AI Humanoid Robotics Book - Embodied Intelligence Platform

**Version:** 1.0.0  
**Date:** January 21, 2025  
**Status:** In Development  
**Project Type:** Educational Platform with AI-Powered Features

---

## 1. Executive Summary

### 1.1 Project Overview
The AI Humanoid Robotics Book is a comprehensive educational platform designed to teach Physical AI and Humanoid Robotics through interactive documentation, hands-on projects, and AI-assisted learning. The platform combines static educational content with dynamic AI capabilities to create an engaging learning experience.

### 1.2 Core Objectives
- Provide structured curriculum for learning humanoid robotics (ROS2, Simulation, NVIDIA Isaac, VLA)
- Enable contextual AI assistance through RAG-powered chatbot
- Support multi-language accessibility for global learners
- Create interactive, searchable documentation with real-time help
- Build a scalable platform for continuous content updates

### 1.3 Target Audience
- University students (undergraduate to graduate level)
- Robotics engineers and researchers
- AI/ML practitioners entering robotics
- Self-learners passionate about embodied intelligence

---

## 2. Technical Architecture

### 2.1 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        Frontend Layer                        │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Docusaurus 3.9.2 (React 19 + TypeScript 5.6)       │  │
│  │  - Documentation Pages (Markdown → React)            │  │
│  │  - Custom Pages (Dashboard, Login, Signup)          │  │
│  │  - AI Chat Interface                                 │  │
│  │  - Translation UI                                    │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            ↓ HTTPS/REST
┌─────────────────────────────────────────────────────────────┐
│                        Backend Layer                         │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  FastAPI (Python 3.11)                               │  │
│  │  - RAG Chat Endpoints (/api/chat/*)                 │  │
│  │  - Translation Service (/api/translate)             │  │
│  │  - Admin Tools (/api/admin/*)                       │  │
│  │  - Health Monitoring                                │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                      AI & Data Layer                         │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────┐  │
│  │   Gemini AI  │  │   Qdrant     │  │   Supabase      │  │
│  │   (Primary)  │  │  (Vectors)   │  │  (PostgreSQL)   │  │
│  │              │  │              │  │                 │  │
│  │  - Chat Gen  │  │  - Embeddings│  │  - User Data    │  │
│  │  - Translate │  │  - RAG Search│  │  - Analytics    │  │
│  └──────────────┘  └──────────────┘  └─────────────────┘  │
│                                                              │
│  ┌──────────────┐                                           │
│  │  Qwen AI     │  (Alternative Provider)                   │
│  │  (Fallback)  │                                           │
│  └──────────────┘                                           │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Technology Stack

#### Frontend Stack
| Component | Technology | Version | Purpose |
|-----------|-----------|---------|---------|
| Framework | Docusaurus | 3.9.2 | Static site generation + React SPA |
| UI Library | React | 19.0.0 | Component-based UI |
| Language | TypeScript | 5.6.2 | Type-safe development |
| Build Tool | Webpack | (via Docusaurus) | Module bundling |
| CSS | Vanilla CSS | - | Styling (CSS Modules) |
| Math Rendering | KaTeX | 0.16.25 | LaTeX equations |
| HTTP Client | Axios | 1.6.2 | API communication |
| Package Manager | npm | - | Dependency management |

#### Backend Stack
| Component | Technology | Version | Purpose |
|-----------|-----------|---------|---------|
| Framework | FastAPI | Latest | REST API server |
| Language | Python | 3.11 | Backend logic |
| AI Provider 1 | Google Gemini | 2.0 Flash | Primary AI model |
| AI Provider 2 | Qwen | Turbo | Fallback AI model |
| Vector DB | Qdrant | Latest | Embedding storage & search |
| Database | Supabase | Latest | User data & analytics |
| Server | Uvicorn | Latest | ASGI server |
| Deployment | Railway/PythonAnywhere | - | Cloud hosting |

#### Development Tools
- **Version Control:** Git + GitHub
- **CI/CD:** GitHub Actions
- **Environment:** Node.js 20.x, Python 3.11
- **Code Quality:** TypeScript compiler, Python type hints
- **Documentation:** OpenAPI/Swagger, Markdown

---

## 3. Feature Specifications

### 3.1 Core Features

#### F1: Documentation System
**Description:** Structured educational content organized into modules

**Technical Details:**
- **Format:** Markdown files with MDX support
- **Structure:** 4 main modules + resources section
  - Module 1: ROS 2 Fundamentals
  - Module 2: Simulation (Gazebo & Unity)
  - Module 3: NVIDIA Isaac Platform
  - Module 4: Vision-Language-Action Models
  - Resources: FAQ, Glossary, References, Tutorials
- **Features:**
  - Sidebar navigation (auto-generated from sidebars.ts)
  - Code syntax highlighting (Prism)
  - Math equation rendering (KaTeX)
  - Image optimization
  - Search functionality (Algolia DocSearch)
  - Responsive design (mobile-first)

**Acceptance Criteria:**
- ✅ All markdown files render correctly
- ✅ Navigation is intuitive and functional
- ✅ Code blocks have proper syntax highlighting
- ✅ Math equations display properly
- ✅ Mobile responsive (< 768px)
- ✅ Page load time < 3 seconds

---

#### F2: RAG-Powered Chatbot
**Description:** AI assistant that answers questions using documentation context

**Technical Details:**
- **Architecture:** Retrieval-Augmented Generation (RAG)
- **Workflow:**
  1. User selects text from documentation
  2. System generates embedding for selected text
  3. Qdrant searches for similar passages (top-k=5)
  4. Context = selected text + retrieved passages
  5. AI generates answer based on context
  6. Response includes source citations

**API Endpoints:**
- `POST /api/chat/general` - General AI chat (no RAG)
- `POST /api/chat/selected` - RAG chat with selected text

**Request/Response Format:**
```json
// Request
{
  "question": "What is ROS2?",
  "context": "ROS 2 is a set of software libraries..."
}

// Response
{
  "answer": "ROS 2 (Robot Operating System 2) is...",
  "sources": "docs/module-1/overview, docs/module-1/ros2-basics"
}
```

**Performance Requirements:**
- Response time: < 5 seconds (95th percentile)
- Accuracy: Answers must be based on provided context
- Rate limiting: 15 requests/minute per user
- Concurrent users: Support 50+ simultaneous users

**Acceptance Criteria:**
- ✅ Chat interface is accessible from all doc pages
- ✅ Selected text is properly captured
- ✅ Relevant passages are retrieved (relevance score > 0.7)
- ✅ Answers are contextually accurate
- ✅ Sources are properly cited
- ✅ Error handling for API failures
- ✅ Loading states are shown during processing

---

#### F3: Multi-Language Translation
**Description:** Translate documentation content to multiple languages

**Supported Languages:**
- Urdu (primary)
- Arabic
- Spanish
- French
- German
- Chinese
- Japanese
- Korean
- Hindi

**Technical Details:**
- **Endpoint:** `POST /api/translate`
- **Preservation Rules:**
  - HTML structure and tags
  - Code blocks (no translation)
  - Technical terms and keywords
  - URLs and file paths
  - Variable/function names

**Request/Response Format:**
```json
// Request
{
  "content": "<h1>Introduction</h1><p>ROS2 is...</p>",
  "target_language": "urdu"
}

// Response
{
  "translated_content": "<h1>تعارف</h1><p>ROS2 ہے...</p>"
}
```

**Acceptance Criteria:**
- ✅ HTML structure is preserved
- ✅ Code blocks remain untranslated
- ✅ Technical terms are handled correctly
- ✅ Translation is grammatically correct
- ✅ Response time < 10 seconds
- ✅ Error handling for unsupported languages

---

#### F4: User Authentication & Dashboard
**Description:** User accounts for personalized experience

**Features:**
- User registration (email + password)
- Login/logout functionality
- User dashboard
  - Learning progress tracking
  - Bookmarked pages
  - Chat history
  - Completed modules

**Technical Details:**
- **Authentication:** Supabase Auth
- **Session Management:** JWT tokens
- **Pages:**
  - `/login` - Login form
  - `/signup` - Registration form
  - `/dashboard` - User dashboard

**Acceptance Criteria:**
- ✅ Users can register with valid email
- ✅ Password validation (min 8 chars, special chars)
- ✅ Login persists across sessions
- ✅ Dashboard shows user-specific data
- ✅ Logout clears session properly

---

#### F5: Document Ingestion System
**Description:** Admin tool to populate vector database with documentation

**Technical Details:**
- **Endpoint:** `POST /api/admin/ingest-docs`
- **Process:**
  1. Scan `docs/` folder for .md files
  2. Parse markdown content
  3. Chunk documents (500-1000 tokens, 200 token overlap)
  4. Generate embeddings using AI provider
  5. Store in Qdrant with metadata
     - source (file path)
     - section (module name)
     - page_number
     - chunk_index

**Metadata Schema:**
```json
{
  "text": "ROS 2 is a set of...",
  "source": "docs/module-1/overview.md",
  "section": "Module 1: ROS 2",
  "page_number": 1,
  "chunk_index": 0,
  "embedding": [0.123, 0.456, ...]
}
```

**Acceptance Criteria:**
- ✅ All markdown files are processed
- ✅ Chunks are properly sized
- ✅ Embeddings are generated successfully
- ✅ Metadata is complete and accurate
- ✅ Process completes in < 10 minutes
- ✅ Error handling for failed files

---

### 3.2 Admin Features

#### F6: Qdrant Status Monitor
**Description:** Check vector database health and statistics

**Endpoint:** `GET /api/admin/qdrant-status`

**Response:**
```json
{
  "status": "initialized",
  "collection_name": "ai-humanoid-robotics-book",
  "points_count": 1523,
  "vector_size": 768,
  "distance": "COSINE"
}
```

---

#### F7: Filesystem Debug Tool
**Description:** Debug deployment issues related to file access

**Endpoint:** `GET /api/admin/filesystem-debug`

**Use Cases:**
- Verify docs/ folder exists on deployment
- Check file permissions
- Troubleshoot ingestion failures

---

## 4. Data Models

### 4.1 Database Schema (Supabase)

#### Users Table
```sql
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  full_name VARCHAR(255),
  created_at TIMESTAMP DEFAULT NOW(),
  last_login TIMESTAMP,
  is_active BOOLEAN DEFAULT TRUE
);
```

#### User Progress Table
```sql
CREATE TABLE user_progress (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  user_id UUID REFERENCES users(id),
  module_id VARCHAR(50),
  page_id VARCHAR(100),
  completed BOOLEAN DEFAULT FALSE,
  last_accessed TIMESTAMP DEFAULT NOW(),
  UNIQUE(user_id, module_id, page_id)
);
```

#### Chat History Table
```sql
CREATE TABLE chat_history (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  user_id UUID REFERENCES users(id),
  question TEXT NOT NULL,
  answer TEXT NOT NULL,
  context TEXT,
  sources TEXT,
  created_at TIMESTAMP DEFAULT NOW()
);
```

### 4.2 Vector Database Schema (Qdrant)

**Collection Name:** `ai-humanoid-robotics-book`

**Vector Configuration:**
- Dimension: 768 (embedding size)
- Distance: Cosine similarity
- Index: HNSW (Hierarchical Navigable Small World)

**Payload Schema:**
```json
{
  "text": "string",           // Document chunk text
  "source": "string",         // File path
  "section": "string",        // Module name
  "page_number": "integer",   // Page number
  "chunk_index": "integer",   // Chunk position
  "metadata": {
    "title": "string",
    "author": "string",
    "date": "string"
  }
}
```

---

## 5. API Specifications

### 5.1 REST API Endpoints

| Method | Endpoint | Description | Auth Required |
|--------|----------|-------------|---------------|
| GET | / | Root endpoint (health check) | No |
| GET | /health | Health check | No |
| POST | /api/chat/general | General AI chat | No |
| POST | /api/chat/selected | RAG chat with context | No |
| POST | /api/translate | Translate content | No |
| POST | /api/admin/ingest-docs | Ingest documents | Yes (Admin) |
| GET | /api/admin/qdrant-status | Check Qdrant status | Yes (Admin) |
| GET | /api/admin/filesystem-debug | Debug filesystem | Yes (Admin) |

### 5.2 Request/Response Standards

**Success Response:**
```json
{
  "status": "success",
  "data": { ... },
  "message": "Operation completed successfully"
}
```

**Error Response:**
```json
{
  "detail": "Error message describing what went wrong"
}
```

**HTTP Status Codes:**
- 200: Success
- 400: Bad request (validation error)
- 401: Unauthorized
- 403: Forbidden
- 404: Not found
- 500: Internal server error
- 503: Service unavailable

---

## 6. Security Specifications

### 6.1 Authentication & Authorization
- **Method:** JWT tokens (Supabase Auth)
- **Token Expiry:** 24 hours
- **Refresh Tokens:** 30 days
- **Password Requirements:**
  - Minimum 8 characters
  - At least 1 uppercase letter
  - At least 1 lowercase letter
  - At least 1 number
  - At least 1 special character

### 6.2 API Security
- **CORS:** Whitelist specific origins
  - http://localhost:3000 (development)
  - https://shaya9.github.io (production)
- **Rate Limiting:** 15 requests/minute per IP
- **Input Validation:** Pydantic models for all inputs
- **SQL Injection Prevention:** Parameterized queries
- **XSS Prevention:** Content sanitization

### 6.3 Data Protection
- **Encryption in Transit:** HTTPS/TLS 1.3
- **Encryption at Rest:** Database-level encryption (Supabase)
- **API Keys:** Environment variables (never committed)
- **Secrets Management:** Railway/PythonAnywhere secrets

---

## 7. Performance Requirements

### 7.1 Response Times
| Operation | Target | Maximum |
|-----------|--------|---------|
| Page Load | < 2s | < 3s |
| API Call (Chat) | < 3s | < 5s |
| API Call (Translate) | < 5s | < 10s |
| Search Query | < 1s | < 2s |
| Document Ingestion | < 5min | < 10min |

### 7.2 Scalability
- **Concurrent Users:** 100+ simultaneous users
- **API Throughput:** 1000+ requests/minute
- **Database:** 10,000+ documents
- **Vector DB:** 100,000+ embeddings

### 7.3 Availability
- **Uptime:** 99.5% (target)
- **Downtime:** < 3.6 hours/month
- **Backup Frequency:** Daily
- **Recovery Time:** < 1 hour

---

## 8. Deployment Specifications

### 8.1 Frontend Deployment (GitHub Pages)
- **Platform:** GitHub Pages
- **Build Command:** `npm run build`
- **Deploy Command:** `npm run deploy`
- **URL:** https://shaya9.github.io/ai-humanoid-robotics-book/
- **CDN:** GitHub's CDN (automatic)

### 8.2 Backend Deployment (Railway)
- **Platform:** Railway
- **Runtime:** Python 3.11
- **Start Command:** `uvicorn main:app --host 0.0.0.0 --port $PORT`
- **Environment:** Production
- **Auto-Deploy:** On push to main branch
- **Health Check:** GET /health

### 8.3 Alternative Backend (PythonAnywhere)
- **Platform:** PythonAnywhere
- **Runtime:** Python 3.11
- **WSGI:** FastAPI with WSGIMiddleware
- **Configuration:** Manual setup via web interface

### 8.4 Environment Variables
```bash
# AI Configuration
AI_PROVIDER=gemini
GEMINI_API_KEY=***
GEMINI_MODEL=gemini-2.0-flash
QWEN_API_KEY=***
QWEN_MODEL=qwen-turbo

# Database
QDRANT_URL=https://***.qdrant.io
QDRANT_API_KEY=***
QDRANT_COLLECTION_NAME=ai-humanoid-robotics-book
SUPABASE_URL=https://***.supabase.co
SUPABASE_KEY=***

# Application
FRONTEND_URL=https://shaya9.github.io/ai-humanoid-robotics-book
CORS_ORIGINS=https://shaya9.github.io,http://localhost:3000
```

---

## 9. Testing Specifications

### 9.1 Frontend Testing
- **Unit Tests:** React components (Jest + React Testing Library)
- **Integration Tests:** API integration (Mock Service Worker)
- **E2E Tests:** User flows (Playwright)
- **Coverage Target:** > 70%

### 9.2 Backend Testing
- **Unit Tests:** Individual functions (pytest)
- **Integration Tests:** API endpoints (pytest + httpx)
- **Load Tests:** Concurrent requests (Locust)
- **Coverage Target:** > 80%

### 9.3 Test Cases

#### Critical Test Cases
1. **Chat Functionality**
   - User selects text → Chat opens
   - Question submitted → Answer received
   - Sources are displayed correctly
   - Error handling for API failures

2. **Translation**
   - HTML structure preserved
   - Code blocks not translated
   - Multiple languages supported
   - Error handling for invalid input

3. **Authentication**
   - User registration successful
   - Login with valid credentials
   - Session persistence
   - Logout clears session

4. **Document Ingestion**
   - All files processed
   - Embeddings generated
   - Metadata stored correctly
   - Error handling for failed files

---

## 10. Monitoring & Analytics

### 10.1 Metrics to Track
- **Performance:**
  - API response times (p50, p95, p99)
  - Page load times
  - Error rates
  - Uptime percentage

- **Usage:**
  - Daily active users
  - Page views per module
  - Chat interactions per day
  - Translation requests per language

- **Business:**
  - User registrations
  - Module completion rates
  - Most viewed pages
  - User retention

### 10.2 Logging
- **Frontend:** Console errors, user actions
- **Backend:** API requests, errors, performance
- **Database:** Query performance, connection pool

### 10.3 Alerting
- **Critical:** API downtime, database failures
- **Warning:** High error rates, slow responses
- **Info:** Deployment success, backup completion

---

## 11. Future Enhancements

### Phase 2 (Q2 2025)
- [ ] Video tutorials integration
- [ ] Interactive code playground
- [ ] Real-time collaboration features
- [ ] Mobile app (React Native)

### Phase 3 (Q3 2025)
- [ ] AI-powered code review
- [ ] Automated assessments
- [ ] Certificate generation
- [ ] Community forum

### Phase 4 (Q4 2025)
- [ ] VR/AR integration for robotics simulation
- [ ] Live instructor sessions
- [ ] Project showcase gallery
- [ ] Job board integration

---

## 12. Maintenance & Support

### 12.1 Regular Maintenance
- **Weekly:** Dependency updates (security patches)
- **Monthly:** Performance optimization, bug fixes
- **Quarterly:** Feature releases, major updates

### 12.2 Support Channels
- **GitHub Issues:** Bug reports, feature requests
- **Discord:** Community support
- **Email:** support@xpertsphere.com

### 12.3 Documentation Updates
- **API Docs:** Updated with each release
- **User Guide:** Updated monthly
- **Developer Docs:** Updated with code changes

---

## 13. Compliance & Legal

### 13.1 Licenses
- **Code:** MIT License
- **Content:** Creative Commons BY-NC-SA 4.0
- **Dependencies:** Various open-source licenses

### 13.2 Privacy
- **GDPR Compliance:** User data handling
- **Cookie Policy:** Analytics cookies only
- **Data Retention:** 2 years (user data), 1 year (logs)

### 13.3 Terms of Use
- Educational use only
- No commercial redistribution
- Attribution required for content reuse

---

## 14. Glossary

| Term | Definition |
|------|------------|
| RAG | Retrieval-Augmented Generation - AI technique combining retrieval and generation |
| Embedding | Vector representation of text for semantic search |
| Vector DB | Database optimized for storing and searching embeddings |
| HNSW | Hierarchical Navigable Small World - efficient similarity search algorithm |
| Chunking | Splitting documents into smaller, overlapping pieces |
| VLA | Vision-Language-Action - AI models that combine visual, language, and action understanding |
| ROS2 | Robot Operating System 2 - middleware for robotics |
| ASGI | Asynchronous Server Gateway Interface - Python web server standard |

---

**Document Version:** 1.0.0  
**Last Updated:** January 21, 2025  
**Maintained By:** Xpertsphere Development Team  
**Contact:** support@xpertsphere.com