#include "dmonitor/rich_text.hpp"

#include <SFML/Graphics/Font.hpp>
#include <SFML/Graphics/Rect.hpp>
#include <SFML/Graphics/RenderTarget.hpp>
#include <SFML/System/String.hpp>

namespace sfe {

void RichText::TextLine::set_font_size(unsigned int size) {
  for (sf::Text &text : texts_)
    text.setCharacterSize(size);

  UpdateGeometry();
}

void RichText::TextLine::set_font(const sf::Font &font) {
  for (sf::Text &text : texts_)
    text.setFont(font);

  UpdateGeometry();
}

void RichText::TextLine::AppendText(sf::Text text) {
  // Set text offset
  UpdateTextAndGeometry(text);

  // Push back
  texts_.push_back(std::move(text));
}

void RichText::TextLine::draw(sf::RenderTarget &target, sf::RenderStates states) const {
  states.transform *= getTransform();

  for (const sf::Text &text : texts_)
    target.draw(text, states);
}

void RichText::TextLine::UpdateGeometry() const {
  bounds_ = sf::FloatRect();

  for (sf::Text &text : texts_)
    UpdateTextAndGeometry(text);
}

void RichText::TextLine::UpdateTextAndGeometry(sf::Text &text) const {
  // Set text offset
  text.setPosition(bounds_.width, 0.f);

  // Update bounds
  int lineSpacing = text.getFont()->getLineSpacing(text.getCharacterSize());
  bounds_.height = std::max(bounds_.height, static_cast<float>(lineSpacing));
  bounds_.width += text.getGlobalBounds().width;
}

RichText::RichText()
    : RichText(nullptr) {

}

RichText::RichText(const sf::Font &font)
    : RichText(&font) {

}

RichText &RichText::operator<<(const sf::Color &color) {
  current_color_ = color;
  return *this;
}

RichText &RichText::operator<<(sf::Text::Style style) {
  current_style_ = style;
  return *this;
}

std::vector<sf::String> explode(const sf::String &string, sf::Uint32 delimiter) {
  if (string.isEmpty())
    return std::vector<sf::String>();

  // For each character in the string
  std::vector<sf::String> result;
  sf::String buffer;
  for (sf::Uint32 character : string) {
    // If we've hit the delimiter character
    if (character == delimiter) {
      // Add them to the result vector
      result.push_back(buffer);
      buffer.clear();
    } else {
      // Accumulate the next character into the sequence
      buffer += character;
    }
  }

  // Add to the result if buffer still has contents or if the last character is a delimiter
  if (!buffer.isEmpty() || string[string.getSize() - 1] == delimiter)
    result.push_back(buffer);

  return result;
}

RichText &RichText::operator<<(const sf::String &string) {
  // Maybe skip
  if (string.isEmpty())
    return *this;

  // Explode into substrings
  std::vector<sf::String> subStrings = explode(string, '\n');

  // Append first substring using the last line
  auto it = subStrings.begin();
  if (it != subStrings.end()) {
    // If there isn't any line, just create it
    if (lines_.empty())
      lines_.resize(1);

    // Remove last line's height
    TextLine &line = lines_.back();
    bounds_.height -= line.global_bounds().height;

    // Append text
    line.AppendText(CreateText(*it));

    // Update bounds
    bounds_.height += line.global_bounds().height;
    bounds_.width = std::max(bounds_.width, line.global_bounds().width);
  }

  // Append the rest of substrings as new lines
  while (++it != subStrings.end()) {
    TextLine line;
    line.setPosition(0.f, bounds_.height);
    line.AppendText(CreateText(*it));
    lines_.push_back(std::move(line));

    // Update bounds
    bounds_.height += line.global_bounds().height;
    bounds_.width = std::max(bounds_.width, line.global_bounds().width);
  }

  // Return
  return *this;
}

void RichText::set_font_size(unsigned int size) {
  // Maybe skip
  if (font_size_ == size)
    return;

  // Update character size
  font_size_ = size;

  // Set texts character size
  for (TextLine &line : lines_)
    line.set_font_size(size);

  UpdateGeometry();
}

void RichText::set_font(const sf::Font &font) {
  // Maybe skip
  if (font_ == &font)
    return;

  // Update font
  font_ = &font;

  // Set texts font
  for (TextLine &line : lines_)
    line.set_font(font);

  UpdateGeometry();
}

void RichText::Clear() {
  // Clear texts
  lines_.clear();

  // Reset bounds
  bounds_ = sf::FloatRect();
}

void RichText::draw(sf::RenderTarget &target, sf::RenderStates states) const {
  states.transform *= getTransform();

  for (const TextLine &line : lines_)
    target.draw(line, states);
}

RichText::RichText(const sf::Font *font)
    : font_(font),
      font_size_(30),
      current_color_(sf::Color::White),
      current_style_(sf::Text::Regular) {

}

sf::Text RichText::CreateText(const sf::String &string) const {
  sf::Text text;
  text.setString(string);
  text.setFillColor(current_color_);
  text.setStyle(current_style_);
  text.setCharacterSize(font_size_);
  if (font_)
    text.setFont(*font_);

  return text;
}

void RichText::UpdateGeometry() const {
  bounds_ = sf::FloatRect();

  for (TextLine &line : lines_) {
    line.setPosition(0.f, bounds_.height);

    bounds_.height += line.global_bounds().height;
    bounds_.width = std::max(bounds_.width, line.global_bounds().width);
  }
}

} // namespace sfe
