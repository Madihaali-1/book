# Data Model: Physical AI and Humanoid Robotics Book

## Entities

### Book Content
- **Name**: Book Content
- **Fields**:
  - title: string (e.g., "Physical AI and Humanoid Robotics")
  - modules: array of Module objects
  - chapters: array of Chapter objects
  - metadata: object containing author, creation date, etc.
- **Relationships**: Contains multiple Modules which contain multiple Chapters
- **Validation**: Title is required, must have exactly 4 modules, each module must have exactly 3 chapters

### Module
- **Name**: Module
- **Fields**:
  - id: string (e.g., "module-1")
  - title: string (e.g., "The Robotic Nervous System (ROS 2)")
  - number: integer (1-4)
  - chapters: array of Chapter objects
  - description: string
- **Relationships**: Belongs to Book Content, contains multiple Chapters
- **Validation**: Must have a unique number between 1-4, title is required, must have exactly 3 chapters

### Chapter
- **Name**: Chapter
- **Fields**:
  - id: string (e.g., "module-1-chapter-1")
  - title: string (e.g., "Introduction to ROS 2 for Physical AI")
  - number: integer (1-3)
  - module_id: string (reference to parent Module)
  - content: string (Markdown content)
  - path: string (URL path for navigation)
- **Relationships**: Belongs to Module
- **Validation**: Must have a unique number between 1-3 within its module, title is required, content is required

### Navigation Structure
- **Name**: Navigation Structure
- **Fields**:
  - modules: array of module identifiers
  - chapters_by_module: object mapping module IDs to arrays of chapter IDs
  - sidebar_config: object containing sidebar configuration
- **Relationships**: References Modules and Chapters
- **Validation**: Must maintain hierarchical integrity between modules and chapters

## State Transitions
- Content can be in "draft", "review", or "published" states
- Navigation structure updates when modules or chapters are added/removed