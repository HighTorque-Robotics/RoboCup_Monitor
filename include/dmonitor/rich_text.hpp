/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file rich_text.hpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-06-07
 */

#pragma once

#include <SFML/Graphics/Transformable.hpp>
#include <SFML/Graphics/Drawable.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/Text.hpp>

#include <SFML/System/Vector2.hpp>

namespace sf {

class Font;
class String;
template<class T>
class Rect;
typedef Rect<float> FloatRect;

} // namespace sf


namespace sfe {

class RichText : public sf::Drawable, public sf::Transformable {
 public:
  /**
   * @brief Nested class that represents a single line text
   */
  class TextLine : public sf::Transformable, public sf::Drawable {
   public:
    /**
     * @brief Set font size
     * @param size - font size
     */
    void set_font_size(unsigned int size);

    /**
     * @brief Set font type
     * @param font - font type
     */
    void set_font(const sf::Font &font);

    /**
     * @brief Get texts
     * @return text of this line
     */
    inline const std::vector<sf::Text> &texts() const {
      return texts_;
    };

    /**
     * @brief Append text
     * @param text - text to append to this line
     */
    void AppendText(sf::Text text);

    /**
     * @brief Get local bounds
     * @return - local bounds
     */
    inline sf::FloatRect local_bounds() const {
      return bounds_;
    };

    /**
     * @brief Get global bounds
     * @return global bounds
     */
    inline sf::FloatRect global_bounds() const {
      return getTransform().transformRect(local_bounds());
    };

   protected:
    /**
     * @brief Draw text line
     * @param target - render target
     * @param states - render status
     */
    void draw(sf::RenderTarget &target, sf::RenderStates states) const override;

   private:
    /**
     * @brief Update geometry
     */
    void UpdateGeometry() const;

    /**
     * @brief Update geometry for a given text
     * @param text - given text
     */
    void UpdateTextAndGeometry(sf::Text &text) const;

    /**
     * @brief List of text for this line
     */
    mutable std::vector<sf::Text> texts_;
    /**
     * @brief Local bounds
     */
    mutable sf::FloatRect bounds_;
  };

  /**
   * @brief Blank constructor
   */
  RichText();

  /**
   * @brief Constructor with given font type
   * @param font
   */
  RichText(const sf::Font &font);

  /**
   * @brief Append text color to current line
   * @param color - text color
   * @return RichText instance with text color appended
   */
  RichText &operator<<(const sf::Color &color);
  /**
   * @brief Append text style to current line
   * @param color - text color
   * @return RichText instance with text style appended
   */
  RichText &operator<<(sf::Text::Style style);
  /**
   * @brief Append texts to current line
   * @param color - text color
   * @return RichText instance with text appended
   */
  RichText &operator<<(const sf::String &string);

  /**
   * @brief Set font size
   * @param size - font size
   */
  void set_font_size(unsigned int size);

  /**
   * @brief Set font
   * @param font - font type
   */
  void set_font(const sf::Font &font);

  /**
   * @brief Clear
   */
  void Clear();

  /**
   * @brief Get text list
   * @return text lines
   */
  inline const std::vector<TextLine> &lines() const {
    return lines_;
  };

  /**
   * @brief Get font size
   * @return font size
   */
  inline unsigned int font_size() const {
    return font_size_;
  };

  /**
   * @brief Get font
   * @return font type
   */
  inline const sf::Font *font() const {
    return font_;
  };

  /**
   * @brief Get local bounds
   * @return local bounds
   */
  inline sf::FloatRect local_bounds() const {

    return bounds_;
  };

  /**
   * @brief Get global bounds
   * @return global bounds
   */
  inline sf::FloatRect global_bounds() const {
    return getTransform().transformRect(local_bounds());
  };

 protected:
  /**
   * @brief Render rich text lines
   * @param target - render target
   * @param states - render states
   */
  void draw(sf::RenderTarget &target, sf::RenderStates states) const override;

 private:
  /**
   * @brief Delegate constructor
   * @param font - font type
   */
  RichText(const sf::Font *font);

  /**
   * @brief Creates a sf::Text instance using the current styles
   * @param string - given text
   * @return sf::Text instance
   */
  sf::Text CreateText(const sf::String &string) const;

  /**
   * @brief Update geometry
   */
  void UpdateGeometry() const;

  /**
   * @brief List of text lines
   */
  mutable std::vector<TextLine> lines_; ///< List of lines
  /**
   * @brief Font for text
   */
  const sf::Font *font_;            ///< Font
  /**
   * @brief Font size
   */
  unsigned int font_size_;      ///< Character size
  /**
   * @brief Local bounds
   */
  mutable sf::FloatRect bounds_;    ///< Local bounds
  /**
   * @brief Current text color
   */
  sf::Color current_color_;          ///< Last used color
  /**
   * @brief Current text style
   */
  sf::Text::Style current_style_;    ///< Last style used
};

} // namespace sfe

namespace dmonitor {

} // namespace dmonitor
