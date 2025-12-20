source "https://rubygems.org"

# GitHub Pages environment (pins Jekyll 3.9.x)
gem "github-pages", "~> 228", group: :jekyll_plugins

# Theme
gem "minima", "~> 2.5"

# Required for Ruby >= 3.4 (csv removed from stdlib)
gem "csv"

# Plugins
group :jekyll_plugins do
  gem "jekyll-feed", "~> 0.12"
end

# Windows / JRuby timezone support
platforms :mingw, :x64_mingw, :mswin, :jruby do
  gem "tzinfo", ">= 1", "< 3"
  gem "tzinfo-data"
end

# File watching (Windows only)
gem "wdm", "~> 0.1.1", platforms: [:mingw, :x64_mingw, :mswin]

# JRuby HTTP parser compatibility
gem "http_parser.rb", "~> 0.6.0", platforms: [:jruby]

# Required since Ruby 3+
gem "webrick", "~> 1.8"

# bundle exec jekyll serve