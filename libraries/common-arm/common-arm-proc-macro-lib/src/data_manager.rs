use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::parse::{Parse, ParseStream};
use syn::{bracketed, parse_macro_input, Data, Ident, Token, Type};

mod kw {
    syn::custom_keyword!(sensors);
}

struct Sensor {
    name: Ident,
    ty: Type,
}

impl Parse for Sensor {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let name: Ident = input.parse()?;
        input.parse::<Token![:]>()?;
        let ty: Type = input.parse()?;

        Ok(Sensor { name, ty })
    }
}

pub struct DataManager {
    sensors_names: Vec<Ident>,
    sensors_types: Vec<Type>,
}

impl Parse for DataManager {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        input.parse::<kw::sensors>()?;
        input.parse::<Token![:]>()?;

        let sensors_raw;
        bracketed!(sensors_raw in input);
        let (sensors_names, sensors_types): (Vec<_>, Vec<_>) = sensors_raw
            .parse_terminated(Sensor::parse, Token![,])?
            .into_iter()
            .map(|x| (x.name, x.ty))
            .unzip();

        Ok(DataManager {
            sensors_names,
            sensors_types,
        })
    }
}

impl DataManager {
    pub fn parse_and_generate(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
        let data_manager = parse_macro_input!(input as DataManager);

        let def = data_manager.def();
        let new_func = data_manager.new_func();
        let handle_data_func = data_manager.handle_data_func();

        (quote! {
            #def

            impl DataManager2 {
                #new_func
                #handle_data_func
            }
        })
        .into()
    }

    fn def(&self) -> TokenStream {
        let mut fields = Vec::new();
        for (name, ty) in self.sensors_names.iter().zip(&self.sensors_types) {
            if let Type::Tuple(x) = ty {
                let elems = x.elems.iter();
                fields.push(quote! {
                    pub #name: (#(Option<#elems>),*)
                });
            } else {
                fields.push(quote! {
                    pub #name: Option<#ty>
                });
            }
        }

        (quote! {
            #[derive(Clone)]
            pub struct DataManager2 {
                #( #fields ),*
            }

        })
    }

    fn new_func(&self) -> TokenStream {
        let mut fields = Vec::new();
        for (name, ty) in self.sensors_names.iter().zip(&self.sensors_types) {
            if let Type::Tuple(x) = ty {
                let elems_count = x.elems.iter().count();
                let none = vec![format_ident!("{}", "None"); elems_count];
                fields.push(quote! {
                    #name: (#( #none ),*)
                });
            } else {
                fields.push(quote! {
                    #name: None
                });
            }
        }

        quote! {
            pub fn new() -> Self {
                Self {
                    #( #fields ),*
                }
            }
        }
    }

    fn handle_data_func(&self) -> TokenStream {
        let mut branches = Vec::new();
        for (name, ty) in self.sensors_names.iter().zip(&self.sensors_types) {
            if let Type::Tuple(tuple_ty) = ty {
                for (i, x) in tuple_ty
                    .elems
                    .iter()
                    .enumerate()
                    .map(|x| (syn::Index::from(x.0), x.1))
                {
                    branches.push(quote! {
                        messages::sensor::SensorData::#x(data) => {
                            self.#name.#i = Some(data)
                        }
                    });
                }
            } else {
                branches.push(quote! {
                    messages::sensor::SensorData::#ty(data) => {
                        self.#name = Some(data)
                    }
                });
            }
        }
        // Ignore any other types
        branches.push(quote! {
           _ => {}
        });

        quote! {
            pub fn handle_data(&mut self, data: Message) {
                match data.data {
                    messages::Data::Sensor(sensor) => match sensor.data {
                        #( #branches ),*
                    },
                    _ => {}
                }
            }
        }
    }
}
