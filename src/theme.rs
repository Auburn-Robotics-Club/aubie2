use vexide::startup::banner::themes::BannerTheme;

#[expect(
    edition_2024_expr_fragment_specifier,
    reason = "OK for this macro to accept `const {}` expressions"
)]
macro_rules! ansi_rgb_bold {
    ($r:expr, $g:expr, $b:expr) => {
        concat!("\x1B[1;38;2;", $r, ";", $g, ";", $b, "m")
    };
}

pub const THEME_WAR_EAGLE: BannerTheme = BannerTheme {
    emoji: "🦅",
    logo_primary: [
        ansi_rgb_bold!(246, 88, 12),
        ansi_rgb_bold!(246, 88, 12),
        ansi_rgb_bold!(246, 88, 12),
        ansi_rgb_bold!(246, 88, 12),
        ansi_rgb_bold!(246, 88, 12),
        ansi_rgb_bold!(246, 88, 12),
        ansi_rgb_bold!(246, 88, 12),
    ],
    logo_secondary: ansi_rgb_bold!(16, 41, 78),
    crate_version: "[1;33m",
    metadata_key: "[1;33m",
};