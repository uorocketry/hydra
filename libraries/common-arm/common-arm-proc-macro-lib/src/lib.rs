use proc_macro::TokenStream;

mod data_manager;


#[proc_macro]
pub fn data_manager(input: TokenStream) -> TokenStream {
    data_manager::DataManager::parse_and_generate(input).into()
}