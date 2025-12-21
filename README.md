# 🤖 AI Humanoid Robotics Book - Embodied Intelligence Platform

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Docusaurus](https://img.shields.io/badge/Docusaurus-3.9.2-green.svg)](https://docusaurus.io/)
[![React](https://img.shields.io/badge/React-19.0.0-blue.svg)](https://reactjs.org/)
[![FastAPI](https://img.shields.io/badge/FastAPI-Latest-009688.svg)](https://fastapi.tiangolo.com/)
[![Python](https://img.shields.io/badge/Python-3.11-blue.svg)](https://www.python.org/)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.6.2-blue.svg)](https://www.typescriptlang.org/)

> **A comprehensive educational platform for learning Physical AI and Humanoid Robotics with AI-powered assistance**

🌐 **Live Demo:** [https://shaya9.github.io/ai-humanoid-robotics-book/](https://shaya9.github.io/ai-humanoid-robotics-book/)  
📚 **Documentation:** [Full Curriculum](https://shaya9.github.io/ai-humanoid-robotics-book/docs/intro)  
🔧 **API Docs:** [OpenAPI Specification](./api-specification.yaml)

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [Architecture](#-architecture)
- [Tech Stack](#-tech-stack)
- [Getting Started](#-getting-started)
- [Project Structure](#-project-structure)
- [Documentation](#-documentation)
- [API Reference](#-api-reference)
- [Deployment](#-deployment)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🎯 Overview

The **AI Humanoid Robotics Book** is an interactive educational platform that teaches Physical AI and Humanoid Robotics through:

- **📖 Structured Curriculum:** 4 comprehensive modules covering ROS2, Simulation, NVIDIA Isaac, and VLA models
- **🤖 AI-Powered Chatbot:** RAG-based assistant for contextual Q&A on documentation
- **🌍 Multi-Language Support:** Translation to 9+ languages including Urdu, Arabic, Spanish, and more
- **💻 Interactive Learning:** Code examples, hands-on projects, and real-world applications
- **📊 Progress Tracking:** User dashboard with learning analytics and achievements

### 🎓 What You'll Learn

1. **Module 1: ROS 2 Fundamentals**
   - ROS2 architecture and core concepts
   - Nodes, topics, services, and actions
   - Launch files and parameters
   - Building robot applications

2. **Module 2: Simulation (Gazebo & Unity)**
   - Physics-based simulation
   - Digital twin creation
   - Gazebo and Unity integration
   - Testing and validation

3. **Module 3: NVIDIA Isaac Platform**
   - Isaac SDK and Isaac Sim
   - GPU-accelerated robotics
   - AI model integration
   - Performance optimization

4. **Module 4: Vision-Language-Action (VLA) Models**
   - Multimodal AI for robotics
   - VLA architecture and training
   - Real-world deployment
   - Future of embodied intelligence

---

## ✨ Features

### 🎨 Frontend Features

- ✅ **Interactive Documentation** - Markdown-based content with MDX support
- ✅ **Code Syntax Highlighting** - Prism.js with multiple language support
- ✅ **Math Rendering** - KaTeX for LaTeX equations
- ✅ **Responsive Design** - Mobile-first, works on all devices
- ✅ **Dark Mode** - Eye-friendly dark theme
- ✅ **Search Functionality** - Quick content discovery
- ✅ **Progress Tracking** - Visual learning progress indicators

### 🤖 AI Features

- ✅ **RAG Chatbot** - Context-aware Q&A using selected text
- ✅ **Dual AI Providers** - Google Gemini 2.0 Flash + Qwen Turbo
- ✅ **Smart Retrieval** - Vector similarity search with Qdrant
- ✅ **Source Citations** - Answers include documentation references
- ✅ **Rate Limiting** - Prevents API abuse (15 req/min)

### 🌐 Translation Features

- ✅ **9+ Languages** - Urdu, Arabic, Spanish, French, German, Chinese, Japanese, Korean, Hindi
- ✅ **HTML Preservation** - Maintains structure and formatting
- ✅ **Code Protection** - Code blocks remain untranslated
- ✅ **Technical Accuracy** - Preserves technical terms

### 👤 User Features

- ✅ **Authentication** - Secure login with Supabase Auth
- ✅ **User Dashboard** - Progress tracking and analytics
- ✅ **Bookmarks** - Save favorite pages
- ✅ **Chat History** - Review past conversations
- ✅ **Certificates** - Completion certificates (coming soon)

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Frontend (Docusaurus)                     │
│  ┌────────────┐  ┌────────────┐  ┌────────────────────┐   │
│  │   Docs     │  │   Pages    │  │   AI Chat UI       │   │
│  │ (Markdown) │  │ (React/TS) │  │   Translation UI   │   │
│  └────────────┘  └────────────┘  └────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                            ↓ REST API
┌─────────────────────────────────────────────────────────────┐
│                    Backend (FastAPI)                         │
│  ┌────────────┐  ┌────────────┐  ┌────────────────────┐   │
│  │  Chat API  │  │ Translate  │  │   Admin Tools      │   │
│  │    RAG     │  │    API     │  │   Doc Ingestion    │   │
│  └────────────┘  └────────────┘  └────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                      Data & AI Layer                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │  Gemini  │  │  Qdrant  │  │ Supabase │  │   Qwen   │  │
│  │   AI     │  │ Vectors  │  │   DB     │  │   AI     │  │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘  │
└─────────────────────────────────────────────────────────────┘
```

---

## 🛠️ Tech Stack

### Frontend
| Technology | Version | Purpose |
|------------|---------|---------|
| Docusaurus | 3.9.2 | Static site generator |
| React | 19.0.0 | UI framework |
| TypeScript | 5.6.2 | Type-safe JavaScript |
| Webpack | (via Docusaurus) | Module bundler |
| Vanilla CSS | - | Styling (CSS Modules) |
| KaTeX | 0.16.25 | Math rendering |
| Axios | 1.6.2 | HTTP client |

### Backend
| Technology | Version | Purpose |
|------------|---------|---------|
| FastAPI | Latest | REST API framework |
| Python | 3.11 | Backend language |
| Google Gemini | 2.0 Flash | Primary AI model |
| Qwen | Turbo | Fallback AI model |
| Qdrant | Latest | Vector database |
| Supabase | Latest | PostgreSQL database |
| Uvicorn | Latest | ASGI server |

### DevOps
| Tool | Purpose |
|------|---------|
| GitHub Actions | CI/CD pipeline |
| GitHub Pages | Frontend hosting |
| Railway | Backend hosting |
| npm | Package management |
| pip | Python packages |

---

## 🚀 Getting Started

### Prerequisites

- **Node.js** 20.x or higher
- **Python** 3.11 or higher
- **npm** or **yarn**
- **Git**

### Installation

#### 1️⃣ Clone the Repository

```bash
git clone https://github.com/SHAYA9/ai-humanoid-robotics-book.git
cd ai-humanoid-robotics-book
```

#### 2️⃣ Frontend Setup

```bash
# Install dependencies
npm install

# Start development server
npm start
```

The frontend will be available at `http://localhost:3000`

#### 3️⃣ Backend Setup

```bash
# Navigate to backend directory
cd backend

# Install dependencies
pip install -r requirements.txt

# Create .env file
cp .env.example .env

# Edit .env with your API keys
# GEMINI_API_KEY=your_key_here
# QDRANT_URL=your_qdrant_url
# QDRANT_API_KEY=your_qdrant_key
# SUPABASE_URL=your_supabase_url
# SUPABASE_KEY=your_supabase_key

# Start backend server
uvicorn main:app --reload --port 8000
```

The backend will be available at `http://localhost:8000`

#### 4️⃣ Ingest Documentation (First Time Only)

```bash
# Run document ingestion script
python scripts/load_docs_to_qdrant.py
```

This will:
- Scan all markdown files in `docs/` folder
- Generate embeddings using AI provider
- Store in Qdrant vector database

---

## 📁 Project Structure

```
ai-humanoid-robotics-book/
├── docs/                          # Documentation content
│   ├── intro.md
│   ├── module-1-ros2/            # ROS2 curriculum
│   ├── module-2-simulation/      # Simulation tutorials
│   ├── module-3-isaac/           # NVIDIA Isaac guides
│   ├── module-4-vla/             # VLA model documentation
│   └── resources/                # FAQ, glossary, references
│
├── src/                          # Frontend source code
│   ├── pages/                    # Custom React pages
│   │   ├── index.tsx            # Landing page
│   │   ├── dashboard.js         # User dashboard
│   │   ├── login.js             # Login page
│   │   └── signup.js            # Registration page
│   ├── css/
│   │   └── custom.css           # Global styles
│   ├── js/
│   │   └── enhancements.js      # UI enhancements
│   └── theme/
│       └── Root.js              # Theme wrapper
│
├── backend/                      # Backend API
│   ├── main.py                  # FastAPI application
│   ├── ai_service.py            # AI provider abstraction
│   ├── gemini_service.py        # Google Gemini implementation
│   ├── qwen_service.py          # Qwen implementation
│   ├── qdrant_service.py        # Vector DB operations
│   └── requirements.txt         # Python dependencies
│
├── scripts/                      # Utility scripts
│   └── load_docs_to_qdrant.py   # Document ingestion
│
├── static/                       # Static assets
│   ├── img/
│   └── manifest.json
│
├── .env                          # Environment variables (create this)
├── .gitignore
├── docusaurus.config.ts          # Docusaurus configuration
├── sidebars.ts                   # Sidebar structure
├── package.json                  # Frontend dependencies
├── tsconfig.json                 # TypeScript configuration
├── api-specification.yaml        # OpenAPI spec
├── SPECIFY.md                    # Technical specification
├── PRD.md                        # Product requirements
├── .clinerules                   # AI assistant rules
└── README.md                     # This file
```

---

## 📚 Documentation

### User Documentation
- **Getting Started:** [docs/intro.md](./docs/intro.md)
- **Module 1 (ROS2):** [docs/module-1-ros2/](./docs/module-1-ros2/)
- **Module 2 (Simulation):** [docs/module-2-simulation/](./docs/module-2-simulation/)
- **Module 3 (Isaac):** [docs/module-3-isaac/](./docs/module-3-isaac/)
- **Module 4 (VLA):** [docs/module-4-vla/](./docs/module-4-vla/)
- **Resources:** [docs/resources/](./docs/resources/)

### Developer Documentation
- **API Specification:** [api-specification.yaml](./api-specification.yaml)
- **Technical Spec:** [SPECIFY.md](./SPECIFY.md)
- **Product Requirements:** [PRD.md](./PRD.md)
- **AI Assistant Rules:** [.clinerules](./.clinerules)
- **Backend README:** [backend/README.md](./backend/README.md)

---

## 🔌 API Reference

### Base URLs
- **Production:** `https://ai-humanoid-robotics-book-production.up.railway.app`
- **Development:** `http://localhost:8000`

### Endpoints

#### Health Check
```http
GET /
GET /health
```

#### Chat Endpoints
```http
POST /api/chat/general
Content-Type: application/json

{
  "question": "What is ROS2?"
}
```

```http
POST /api/chat/selected
Content-Type: application/json

{
  "question": "Explain this concept",
  "context": "ROS 2 is a set of software libraries..."
}
```

#### Translation
```http
POST /api/translate
Content-Type: application/json

{
  "content": "<h1>Introduction</h1>",
  "target_language": "urdu"
}
```

#### Admin Endpoints
```http
POST /api/admin/ingest-docs
GET /api/admin/qdrant-status
GET /api/admin/filesystem-debug
```

For complete API documentation, see [api-specification.yaml](./api-specification.yaml)

---

## 🌐 Deployment

### Frontend Deployment (GitHub Pages)

```bash
# Build production bundle
npm run build

# Deploy to GitHub Pages
npm run deploy
```

### Backend Deployment (Railway)

1. Create new project on [Railway](https://railway.app/)
2. Connect GitHub repository
3. Add environment variables:
   - `GEMINI_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `SUPABASE_URL`
   - `SUPABASE_KEY`
   - `FRONTEND_URL`
4. Deploy automatically on push to `main` branch

### Alternative Backend (PythonAnywhere)

1. Create account on [PythonAnywhere](https://www.pythonanywhere.com/)
2. Upload code via Git or file manager
3. Configure WSGI file
4. Set environment variables
5. Reload web app

---

## 🤝 Contributing

We welcome contributions! Please follow these steps:

1. **Fork the repository**
2. **Create a feature branch**
   ```bash
   git checkout -b feature/amazing-feature
   ```
3. **Make your changes**
4. **Commit with descriptive message**
   ```bash
   git commit -m "Add amazing feature"
   ```
5. **Push to your fork**
   ```bash
   git push origin feature/amazing-feature
   ```
6. **Open a Pull Request**

### Contribution Guidelines

- Follow existing code style and conventions
- Add tests for new features
- Update documentation as needed
- Ensure all tests pass before submitting PR
- Write clear, descriptive commit messages

### Code of Conduct

Please be respectful and constructive in all interactions. We aim to create a welcoming environment for all contributors.

---

## 📝 License

This project is licensed under the **MIT License** - see the [LICENSE](./LICENSE) file for details.

### Content License

Documentation content is licensed under **Creative Commons BY-NC-SA 4.0**:
- ✅ Share and adapt the content
- ✅ Give appropriate credit
- ❌ No commercial use
- ❌ Share under same license

---

## 👥 Team

**Maintained by:** Xpertsphere Team  
**Project Lead:** SHAYAN  
**Contact:** support@xpertsphere.com  
**Website:** [https://xpertsphere.vercel.app](https://xpertsphere.vercel.app)

---

## 🙏 Acknowledgments

- **Docusaurus** - Amazing documentation framework
- **FastAPI** - Modern Python web framework
- **Google Gemini** - Powerful AI capabilities
- **Qdrant** - Efficient vector database
- **Supabase** - Backend-as-a-Service platform
- **Open Source Community** - For all the amazing tools

---

## 📊 Project Status

- ✅ **MVP Complete** - Core features implemented
- 🚧 **In Development** - Additional features being added
- 📅 **Next Release:** Q2 2025 (Video tutorials, code playground)

### Roadmap

- [x] Documentation system
- [x] RAG chatbot
- [x] Multi-language translation
- [x] User authentication
- [x] Basic dashboard
- [ ] Interactive code playground
- [ ] Video tutorials
- [ ] Community forum
- [ ] Mobile app
- [ ] VR/AR integration

---

## 📞 Support

- **GitHub Issues:** [Report bugs or request features](https://github.com/SHAYA9/ai-humanoid-robotics-book/issues)
- **Discord:** [Join our community](https://discord.gg/robotics)
- **Email:** support@xpertsphere.com
- **Twitter:** [@airoboticsbook](https://twitter.com/airoboticsbook)

---

## ⭐ Star History

If you find this project helpful, please consider giving it a star! ⭐

[![Star History Chart](https://api.star-history.com/svg?repos=SHAYA9/ai-humanoid-robotics-book&type=Date)](https://star-history.com/#SHAYA9/ai-humanoid-robotics-book&Date)

---

**Made with ❤️ by the Xpertsphere Team**

*Empowering the next generation of robotics engineers through accessible, AI-powered education.*