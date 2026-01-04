"use client";

import React, { useState, useMemo } from 'react';
import { ProjectBlurb } from './ProjectCard';

interface ProjectControlsProps {
  allProjects: ProjectBlurb[];
  onFilterChange: (filteredProjects: ProjectBlurb[]) => void;
}

export function ProjectControls({ allProjects, onFilterChange }: ProjectControlsProps) {
  const [searchTerm, setSearchTerm] = useState('');
  const [selectedTags, setSelectedTags] = useState<string[]>([]);
  const [sortOrder, setSortOrder] = useState<'date-desc' | 'date-asc' | 'title-asc' | 'title-desc'>('date-desc');

  const uniqueTags = useMemo(() => {
    const tags = new Set<string>();
    allProjects.forEach(project => {
      project.tags.forEach(tag => tags.add(tag));
    });
    return Array.from(tags).sort();
  }, [allProjects]);

  React.useEffect(() => {
    let filtered = [...allProjects];

    if (searchTerm) {
      filtered = filtered.filter(project =>
        project.title.toLowerCase().includes(searchTerm.toLowerCase()) ||
        project.description.toLowerCase().includes(searchTerm.toLowerCase())
      );
    }

    if (selectedTags.length > 0) {
      filtered = filtered.filter(project =>
        selectedTags.every(tag => project.tags.includes(tag))
      );
    }

    switch (sortOrder) {
      case 'date-desc':
        filtered.sort((a, b) => new Date(b.date || 0).getTime() - new Date(a.date || 0).getTime());
        break;
      case 'date-asc':
        filtered.sort((a, b) => new Date(a.date || 0).getTime() - new Date(b.date || 0).getTime());
        break;
      case 'title-asc':
        filtered.sort((a, b) => a.title.localeCompare(b.title));
        break;
      case 'title-desc':
        filtered.sort((a, b) => b.title.localeCompare(a.title));
        break;
    }

    onFilterChange(filtered);
  }, [searchTerm, selectedTags, sortOrder, allProjects, onFilterChange]);

  const handleTagClick = (tag: string) => {
    setSelectedTags(prevTags =>
      prevTags.includes(tag) ? prevTags.filter(t => t !== tag) : [...prevTags, tag]
    );
  };

  return (
    <div className="py-4 space-y-4 bg-background/80 backdrop-blur-md sticky top-0 z-10">
      {/* Search and Sort Row */}
      <div className="flex flex-col sm:flex-row gap-4">
        <input
          type="text"
          placeholder="Search projects..."
          className="flex-grow p-3 border border-secondary rounded-xl bg-background text-text placeholder:text-gray-500 focus:outline-none focus:border-[#4FA3E8]/60 transition-all duration-300"
          value={searchTerm}
          onChange={(e) => setSearchTerm(e.target.value)}
        />
        <select
          className="p-3 border border-secondary rounded-xl bg-background text-text focus:outline-none focus:border-[#4FA3E8]/60 transition-all duration-300 sm:w-48"
          value={sortOrder}
          onChange={(e) => setSortOrder(e.target.value as typeof sortOrder)}
        >
          <option value="date-desc">Newest First</option>
          <option value="date-asc">Oldest First</option>
          <option value="title-asc">Title (A-Z)</option>
          <option value="title-desc">Title (Z-A)</option>
        </select>
      </div>

      {/* Tag Filters - Inline */}
      <div className="flex flex-wrap items-center gap-2">
        <span className="text-sm text-textMuted">Filter:</span>
        {uniqueTags.map(tag => (
          <button
            key={tag}
            onClick={() => handleTagClick(tag)}
            className={`px-3 py-1 border rounded-full text-xs transition-all duration-300
              ${selectedTags.includes(tag) 
                ? 'bg-[#4FA3E8]/20 text-[#4FA3E8] border-[#4FA3E8] shadow-[0_0_10px_rgba(79,163,232,0.3)]' 
                : 'bg-background border-gray-600 text-text hover:border-[#4FA3E8]/60 hover:text-[#4FA3E8]'}`}
          >
            {tag}
          </button>
        ))}
        {selectedTags.length > 0 && (
          <button
            onClick={() => setSelectedTags([])}
            className="px-3 py-1 text-xs text-textMuted hover:text-[#4FA3E8] transition-colors duration-300"
          >
            Clear all ×
          </button>
        )}
      </div>
    </div>
  );
}