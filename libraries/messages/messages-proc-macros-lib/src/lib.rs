use proc_macro::TokenStream;
use quote::quote;

/// Simple macro to easily add derives that are common for the messages crates.
#[proc_macro_attribute]
pub fn common_derives(args: TokenStream, input: TokenStream) -> TokenStream {
    let mut output = TokenStream::from(quote! {
        #[derive(serde::Serialize, serde::Deserialize, Clone, Debug)]
        #[cfg_attr(feature = "ts", derive(ts_rs::TS))]
        #[cfg_attr(feature = "ts", ts(export))]
        #[cfg_attr(test, derive(proptest_derive::Arbitrary))]
    });

    // Allow to omit the defmt::Format derive. Useful if this should be manually implemented.
    if !args.to_string().contains("NoFormat") {
        output.extend(TokenStream::from(quote! {
            #[derive(defmt::Format)]
        }));
    }

    output.extend(input);
    output
}
